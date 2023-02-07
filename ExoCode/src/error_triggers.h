#ifndef ERROR_TRIGGERS_H
#define ERROR_TRIGGERS_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "Exo.h"
#include "error_types.h"

#include "Arduino.h"
#include "Utilities.h"
#include "Config.h"
#include "ParseIni.h"

// TODO: Make these device configuration dependent

namespace error_triggers_state
{
    bool triggered_error = false;

    // Torque goodies
    const float torque_output_alpha = 0.2;
    const float torque_output_threshold = 45;
    float average_torque_output_left = 0;
    float average_torque_output_right = 0;

    // Torque sensor goodies
    const int std_dev_multiple = 10;
    const float window_size = 100;
    std::queue<float> torque_sensor_queue_left;
    std::queue<float> torque_sensor_queue_right;
    int failure_count_left = 0;
    int failure_count_right = 0;
    const int failure_count_threshold = 2;

    // Tracking goodies
    const float tracking_alpha = 0.1;
    const float tracking_threshold = 15; // Nm
    float average_tracking_error_left = 0;
    float average_tracking_error_right = 0;

    // Transimission efficiency goodies
    const float transmission_efficiency_alpha = 0.1;
    const float transmission_efficiency_threshold = 0.01;
    float average_motor_torque_left = 0;
    float average_motor_torque_right = 0;

    // PJMC State goodies
    // alpha = 2 / (N + 1), where N is the equivalent moving average window size
    const float pjmc_state_time = 10; // seconds
    const float pjmc_state_alpha = 2 / ((pjmc_state_time * LOOP_FREQ_HZ) + 1);
    const float pjmc_state_threshold = 0.95;
    float average_pjmc_state_left = 0;
    float average_pjmc_state_right = 0;
}

// Returns true if error
namespace error_triggers
{
    int soft(Exo* exo, ExoData* exo_data)
    {
        if (error_triggers_state::triggered_error)
            return NO_ERROR;
        return NO_ERROR;
    }
    
    int hard(Exo* exo, ExoData* exo_data)
    {
        if (error_triggers_state::triggered_error)
            return NO_ERROR;
        // Check if the legs have been in stance for too long
        error_triggers_state::average_pjmc_state_left = utils::ewma(exo_data->left_leg.toe_stance,
                        error_triggers_state::average_pjmc_state_left, error_triggers_state::pjmc_state_alpha);
        error_triggers_state::average_pjmc_state_right = utils::ewma(exo_data->right_leg.toe_stance,
                        error_triggers_state::average_pjmc_state_right, error_triggers_state::pjmc_state_alpha);

        if ((error_triggers_state::average_pjmc_state_left > error_triggers_state::pjmc_state_threshold) ||
            (error_triggers_state::average_pjmc_state_right > error_triggers_state::pjmc_state_threshold))
        {
            //Serial.println("Error: PJMC State too high");
            error_triggers_state::triggered_error = true;
            return POOR_STATE_VARIANCE;
        }
        
        return NO_ERROR;
    }

    int fatal(Exo* exo, ExoData* exo_data)
    {
        if (error_triggers_state::triggered_error)
            return NO_ERROR;

        // Only run for configurations with torque sensors
        if (exo_data->config[config_defs::exo_name_idx] == (uint8_t)config_defs::exo_name::bilateral_ankle ||
            exo_data->config[config_defs::exo_name_idx] == (uint8_t)config_defs::exo_name::bilateral_hip_ankle)
        {
            // Check if the torque is too high
            error_triggers_state::average_torque_output_left = utils::ewma(exo_data->left_leg.ankle.torque_reading,
                            error_triggers_state::average_torque_output_left, error_triggers_state::torque_output_alpha);
            error_triggers_state::average_torque_output_right = utils::ewma(exo_data->right_leg.ankle.torque_reading,
                            error_triggers_state::average_torque_output_right, error_triggers_state::torque_output_alpha);
            
            if ((abs(error_triggers_state::average_torque_output_left) > error_triggers_state::torque_output_threshold) ||
                (abs(error_triggers_state::average_torque_output_right) > error_triggers_state::torque_output_threshold))
            {
                error_triggers_state::triggered_error = true;
                return TORQUE_OUT_OF_BOUNDS;
            }

            // Check if the tracking error is too high
            float tracking_error_left = exo_data->left_leg.ankle.torque_reading - exo_data->left_leg.ankle.controller.ff_setpoint;
            float tracking_error_right = exo_data->right_leg.ankle.torque_reading - exo_data->right_leg.ankle.controller.ff_setpoint;
            error_triggers_state::average_tracking_error_left = utils::ewma(tracking_error_left,
                            error_triggers_state::average_tracking_error_left, error_triggers_state::tracking_alpha);
            error_triggers_state::average_tracking_error_right = utils::ewma(tracking_error_right,
                            error_triggers_state::average_tracking_error_right, error_triggers_state::tracking_alpha);
            
            if ((abs(error_triggers_state::average_tracking_error_left) > error_triggers_state::tracking_threshold) ||
                (abs(error_triggers_state::average_tracking_error_right) > error_triggers_state::tracking_threshold))
            {
                error_triggers_state::triggered_error = true;
                return TRACKING_ERROR;
            }


            // Check the torque sensor variance, if the variance is too high, then the sensor may be faulty
            error_triggers_state::torque_sensor_queue_left.push(exo_data->left_leg.ankle.torque_reading);
            error_triggers_state::torque_sensor_queue_right.push(exo_data->right_leg.ankle.torque_reading);
            if (error_triggers_state::torque_sensor_queue_left.size() > error_triggers_state::window_size) 
            {
                error_triggers_state::torque_sensor_queue_left.pop();
                error_triggers_state::torque_sensor_queue_right.pop();

                std::pair<float, float> left_population_vals = utils::online_std_dev(error_triggers_state::torque_sensor_queue_left);
                std::pair<float, float> right_population_vals = utils::online_std_dev(error_triggers_state::torque_sensor_queue_right);
                std::pair<float, float> left_bounds = std::make_pair(left_population_vals.first - error_triggers_state::std_dev_multiple*left_population_vals.second,
                                                                    left_population_vals.first + error_triggers_state::std_dev_multiple*left_population_vals.second);
                std::pair<float, float> right_bounds = std::make_pair(right_population_vals.first - error_triggers_state::std_dev_multiple*right_population_vals.second,
                                                                    right_population_vals.first + error_triggers_state::std_dev_multiple*right_population_vals.second);

                error_triggers_state::failure_count_left += float(utils::is_outside_range(exo_data->left_leg.ankle.torque_reading, left_bounds.first, left_bounds.second));
                error_triggers_state::failure_count_right += float(utils::is_outside_range(exo_data->right_leg.ankle.torque_reading, right_bounds.first, right_bounds.second));

                if (error_triggers_state::failure_count_left > error_triggers_state::failure_count_threshold ||
                    error_triggers_state::failure_count_right > error_triggers_state::failure_count_threshold)
                {
                    error_triggers_state::triggered_error = true;
                    return TORQUE_VARIANCE_ERROR;
                }
            }
            

            // Check the transmission efficiency. If its too low, the cable may be broken. Must low pass motor current to account for time delay
            // float left_motor_torque = abs(exo_data->left_leg.ankle.motor.i) * exo->left_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::left_ankle);
            // error_triggers_state::average_motor_torque_left = utils::ewma(abs(left_motor_torque),
            //                 error_triggers_state::average_motor_torque_left, error_triggers_state::transmission_efficiency_alpha); 
            // float right_motor_torque = abs(exo_data->right_leg.ankle.motor.i) * exo->right_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::right_ankle);
            // error_triggers_state::average_motor_torque_right = utils::ewma(abs(right_motor_torque),
            //                 error_triggers_state::average_motor_torque_right, error_triggers_state::transmission_efficiency_alpha);
            
            // float left_transmission_efficiency = 1;
            // float right_transmission_efficiency = 1;
            // if (!utils::is_close_to(error_triggers_state::average_motor_torque_left, 0, 0.001))
            // {
            //     left_transmission_efficiency = exo_data->left_leg.ankle.torque_reading / error_triggers_state::average_motor_torque_left;
            // }
            // if (!utils::is_close_to(error_triggers_state::average_motor_torque_right, 0, 0.001))
            // {
            //     right_transmission_efficiency = exo_data->right_leg.ankle.torque_reading / error_triggers_state::average_motor_torque_right;
            // }

            // if ((abs(left_transmission_efficiency) < error_triggers_state::transmission_efficiency_threshold) ||
            //     (abs(right_transmission_efficiency) < error_triggers_state::transmission_efficiency_threshold))
            // {
            //     Serial.println("Error: Transmission efficiency too low");
            //     return POOR_TRANSMISSION_EFFICIENCY;
            // }
        }
        
        return NO_ERROR;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)