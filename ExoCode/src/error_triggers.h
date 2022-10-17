#ifndef ERROR_TRIGGERS_H
#define ERROR_TRIGGERS_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "Exo.h"
#include "error_types.h"

#include "Arduino.h"
#include "Utilities.h"
#include "Config.h"

namespace error_triggers_state
{
    // PID goodies
    const float pid_output_alpha = 0.1;
    const float pid_output_threshold = 2000;
    float average_pid_output_left = 0;
    float average_pid_output_right = 0;

    // PJMC State goodies
    // alpha = 2 / (N + 1), where N is the equivalent moving average window size
    const float pjmc_state_time = 2; // seconds
    const float pjmc_state_alpha = 0.0001;//2 / ((pjmc_state_time * LOOP_FREQ_HZ) + 1);
    const float pjmc_state_threshold = 0.975;
    float average_pjmc_state_left = 0;
    float average_pjmc_state_right = 0;

}

// Returns true if error
namespace error_triggers
{
    uint8_t soft(Exo* exo, ExoData* exo_data)
    {
        
        return false;
    }
    uint8_t hard(Exo* exo, ExoData* exo_data)
    {
        // // Check if the legs have been in stance for too long
        // error_triggers_state::average_pjmc_state_left = utils::ewma(error_triggers_state::average_pjmc_state_left,
        //                 exo_data->left_leg.toe_stance, error_triggers_state::pjmc_state_alpha);
        // error_triggers_state::average_pjmc_state_right = utils::ewma(error_triggers_state::average_pjmc_state_right,
        //                 exo_data->right_leg.toe_stance, error_triggers_state::pjmc_state_alpha);

        // static uint32_t hard_error_count = 0;
        // hard_error_count++;
        // if (hard_error_count > 100)
        // {
        //     hard_error_count = 0;
        //     Serial.print("Right Raw PJMC State: ");
        //     Serial.print(exo_data->right_leg.toe_stance);
        //     Serial.print("\t");
        //     Serial.print("Right PJMC State: ");
        //     Serial.println(error_triggers_state::average_pjmc_state_right);
        // }

        // if ((error_triggers_state::average_pjmc_state_left > error_triggers_state::pjmc_state_threshold) ||
        //     (error_triggers_state::average_pjmc_state_right > error_triggers_state::pjmc_state_threshold))
        // {
        //     return true;
        // }
        
        return false;
    }
    uint8_t fatal(Exo* exo, ExoData* exo_data)
    {
        // Check if the PID output is too high
        error_triggers_state::average_pid_output_left = utils::ewma(error_triggers_state::average_pid_output_left, 
                        exo_data->left_leg.ankle.controller.filtered_cmd, error_triggers_state::pid_output_alpha);
        error_triggers_state::average_pid_output_right = utils::ewma(error_triggers_state::average_pid_output_right,
                        exo_data->right_leg.ankle.controller.filtered_cmd, error_triggers_state::pid_output_alpha);
        // static uint32_t fatal_error_count = 0;
        // fatal_error_count++;
        // if (fatal_error_count > 100)
        // {
        //     fatal_error_count = 0;
        //     Serial.print("Left PID Output: ");
        //     Serial.println(error_triggers_state::average_pid_output_left);
        //     Serial.print("Right PID Output: ");
        //     Serial.println(error_triggers_state::average_pid_output_right);
        // }
        
        if ((abs(error_triggers_state::average_pid_output_left) > error_triggers_state::pid_output_threshold) ||
            (abs(error_triggers_state::average_pid_output_right) > error_triggers_state::pid_output_threshold))
        {
            return true;
        }

        // TODO: Add more fatal triggers: Torque limit, tracking error, etc.
        return false;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)