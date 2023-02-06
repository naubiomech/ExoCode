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
    // Torque goodies
    const float torque_output_alpha = 0.2;
    const float torque_output_threshold = 45;
    float average_torque_output_left = 0;
    float average_torque_output_right = 0;

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
        return NO_ERROR;
    }
    int hard(Exo* exo, ExoData* exo_data)
    {
        // Check if the legs have been in stance for too long
        error_triggers_state::average_pjmc_state_left = utils::ewma(exo_data->left_leg.toe_stance,
                        error_triggers_state::average_pjmc_state_left, error_triggers_state::pjmc_state_alpha);
        error_triggers_state::average_pjmc_state_right = utils::ewma(exo_data->right_leg.toe_stance,
                        error_triggers_state::average_pjmc_state_right, error_triggers_state::pjmc_state_alpha);

        // static uint32_t hard_error_count = 0;
        // hard_error_count++;
        // if (hard_error_count > 250)
        // {
        //     hard_error_count = 0;
        //     Serial.print("Right Raw PJMC State: ");
        //     Serial.print(exo_data->right_leg.toe_stance);
        //     Serial.print("\t");
        //     Serial.print("Right PJMC State: ");
        //     Serial.print(error_triggers_state::average_pjmc_state_right);
        //     Serial.print("\t");
        //     Serial.print("Left Raw PJMC State: ");
        //     Serial.print(exo_data->left_leg.toe_stance);
        //     Serial.print("\t");
        //     Serial.print("Left PJMC State: ");
        //     Serial.println(error_triggers_state::average_pjmc_state_left);
        // }

        if ((error_triggers_state::average_pjmc_state_left > error_triggers_state::pjmc_state_threshold) ||
            (error_triggers_state::average_pjmc_state_right > error_triggers_state::pjmc_state_threshold))
        {
            //Serial.println("Error: PJMC State too high");
            return POOR_STATE_VARIANCE;
        }
        
        return NO_ERROR;
    }
    int fatal(Exo* exo, ExoData* exo_data)
    {
        // Check if the torque is too high
        error_triggers_state::average_torque_output_left = utils::ewma(exo_data->left_leg.ankle.torque_reading,
                        error_triggers_state::average_torque_output_left, error_triggers_state::torque_output_alpha);
        error_triggers_state::average_torque_output_right = utils::ewma(exo_data->right_leg.ankle.torque_reading,
                        error_triggers_state::average_torque_output_right, error_triggers_state::torque_output_alpha);
        
        if ((abs(error_triggers_state::average_torque_output_left) > error_triggers_state::torque_output_threshold) ||
            (abs(error_triggers_state::average_torque_output_right) > error_triggers_state::torque_output_threshold))
        {
            return TORQUE_OUT_OF_BOUNDS;
        }

        // Ckeck if the tracking error is too high
        float tracking_error_left = exo_data->left_leg.ankle.torque_reading - exo_data->left_leg.ankle.controller.ff_setpoint;
        float tracking_error_right = exo_data->right_leg.ankle.torque_reading - exo_data->right_leg.ankle.controller.ff_setpoint;
        error_triggers_state::average_tracking_error_left = utils::ewma(tracking_error_left,
                        error_triggers_state::average_tracking_error_left, error_triggers_state::tracking_alpha);
        error_triggers_state::average_tracking_error_right = utils::ewma(tracking_error_right,
                        error_triggers_state::average_tracking_error_right, error_triggers_state::tracking_alpha);
        
        if ((abs(error_triggers_state::average_tracking_error_left) > error_triggers_state::tracking_threshold) ||
            (abs(error_triggers_state::average_tracking_error_right) > error_triggers_state::tracking_threshold))
        {
            //Serial.println("Error: Tracking error too high");
            return TRACKING_ERROR;
        }

        // Check the transmission efficiency. If its too low, the cable may be broken. Must low pass motor current to account for time delay
        float left_motor_torque = exo_data->left_leg.ankle.motor.i * exo->left_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::left_ankle);
        error_triggers_state::average_motor_torque_left = utils::ewma(left_motor_torque,
                        error_triggers_state::average_motor_torque_left, error_triggers_state::transmission_efficiency_alpha); 
        float right_motor_torque = exo_data->right_leg.ankle.motor.i * exo->right_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::right_ankle);
        error_triggers_state::average_motor_torque_right = utils::ewma(right_motor_torque,
                        error_triggers_state::average_motor_torque_right, error_triggers_state::transmission_efficiency_alpha);
        
        float left_transmission_efficiency = 1;
        float right_transmission_efficiency = 1;
        if (!utils::is_close_to(error_triggers_state::average_motor_torque_left, 0, 0.001))
        {
            left_transmission_efficiency = exo_data->left_leg.ankle.torque_reading / error_triggers_state::average_motor_torque_left;
        }
        if (!utils::is_close_to(error_triggers_state::average_motor_torque_right, 0, 0.001))
        {
            right_transmission_efficiency = exo_data->right_leg.ankle.torque_reading / error_triggers_state::average_motor_torque_right;
        }

        // print the transmission efficiency
        // static uint32_t transmission_efficiency_count = 0;
        // transmission_efficiency_count++;
        // if (transmission_efficiency_count > 2000) {
        //     transmission_efficiency_count = 0;
        //     Serial.print("Left Ankle Torque: ");
        //     Serial.print(exo_data->left_leg.ankle.torque_reading);
        //     Serial.print("\t");
        //     Serial.print("Left Torque: ");
        //     Serial.print(left_motor_torque);
        //     Serial.print("\t");
        //     Serial.print("Left Transmission Efficiency: ");
        //     Serial.print(left_transmission_efficiency);
        //     Serial.print("\t");
        //     Serial.print("Right Ankle Torque: ");
        //     Serial.print(exo_data->right_leg.ankle.torque_reading);
        //     Serial.print("\t");
        //     Serial.print("Right Torque: ");
        //     Serial.print(right_motor_torque);
        //     Serial.print("\t");
        //     Serial.print("Right Transmission Efficiency: ");
        //     Serial.println(right_transmission_efficiency);
        // }

        if ((left_transmission_efficiency < error_triggers_state::transmission_efficiency_threshold) ||
            (right_transmission_efficiency < error_triggers_state::transmission_efficiency_threshold))
        {
            //Serial.println("Error: Transmission efficiency too low");
            //return POOR_TRANSMISSION_EFFICIENCY;
        }
        
        return NO_ERROR;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)