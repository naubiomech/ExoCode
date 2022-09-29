/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Exo.h"
#include "Time_Helper.h"
#include "UARTHandler.h"
#include "UART_msg_t.h"
#include "uart_commands.h"

//#define EXO_DEBUG 1

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
/*
 * Constructor for the Exo
 * Takes the exo_data
 * Uses initializer list for legs.
 * Only stores these objects, and exo_data pointer.
 */
Exo::Exo(ExoData* exo_data)
: left_leg(true, exo_data)
, right_leg(false, exo_data) // constructor: uses initializer list for the legs.
, sync_led(logic_micro_pins::sync_led_pin, sync_time::SYNC_START_STOP_HALF_PERIOD_US, sync_time::SYNC_HALF_PERIOD_US, logic_micro_pins::sync_led_on_state, logic_micro_pins::sync_default_pin)  // Create a sync LED object, the first and last arguments (pin) are found in Board.h, and the rest are in Config.h.  If you do not have a digital input for the default state you can remove SYNC_DEFAULT_STATE_PIN.  
, status_led(logic_micro_pins::status_led_r_pin, logic_micro_pins::status_led_g_pin, logic_micro_pins::status_led_b_pin)  // Create the status LED object. 
#ifdef USE_SPEED_CHECK
    ,speed_check(logic_micro_pins::speed_check_pin)
#endif
{
    this->data = exo_data;
    #ifdef EXO_DEBUG
        Serial.println("Exo :: Constructor : _data set");
    #endif
    pinMode(logic_micro_pins::motor_stop_pin,INPUT_PULLUP);
    #ifdef EXO_DEBUG
        Serial.println("Exo :: Constructor : motor_stop_pin Mode set");
    #endif
};

/* 
 * Run the exo 
 */
void Exo::run()
{
    // Check if we are within the system frequency we want.
    static UARTHandler* handler = UARTHandler::get_instance();
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static float context = t_helper->generate_new_context();
    static float delta_t = 0;
    static uint16_t prev_status = data->status;
    delta_t += t_helper->tick(context);

    // Check if the real time data is ready to be sent.
    static float rt_context = t_helper->generate_new_context();
    static float rt_delta_t = 0;

    static uint8_t should_plot = 0;
    
    if (((delta_t <= ((float) 1/LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE))) && (delta_t >= ((float)1 / LOOP_FREQ_HZ * 1000000 * (1 - LOOP_TIME_TOLERANCE)))))
    {

        #ifdef USE_SPEED_CHECK
            speed_check.toggle();
        #endif

        // check if trial went from off to on
        // if (data->status != prev_status && (data->status == status_defs::messages::trial_on || data->status == status_defs::messages::trial_off)) {
        //     if (prev_status == status_defs::messages::trial_on && data->status == status_defs::messages::trial_off) 
        //     {
        //         // from on to off
        //         Serial.println("Exo::run: Trial Off");
        //         should_plot = 0;
        //     }
        //     else if (prev_status == status_defs::messages::trial_off && data->status == status_defs::messages::trial_on) 
        //     {
        //         // from off to on
        //         Serial.println("Exo::run: Trial On");
        //         should_plot = 1;
        //     }
        //     prev_status = data->status;
        // }

        // check if we should update the sync LED and record the LED on/off state.
        data->sync_led_state = sync_led.handler();
        bool trial_running = sync_led.get_is_blinking();

        // check the estop
        data->estop = digitalRead(logic_micro_pins::motor_stop_pin);

        // Serial.print("Exo::run: is error : ");
        // Serial.print(((data->status & status_defs::messages::error) == status_defs::messages::error));
        // Serial.print("\n");
        // if (trial_running && ((data->status != status_defs::messages::error) && (data->status != status_defs::messages::test)))
        // {
        //     data->status = status_defs::messages::trial_on;
        // }
        // else if ((!trial_running) && ((data->status != status_defs::messages::error) && (data->status != status_defs::messages::test)))
        // {
        //     data->status = status_defs::messages::trial_off;
        // }
        // else
        // {
        //     // Serial.print("Exo::run:Error or Test\n");
        // }

        // Record the leg data and send new commands to the motors.
        left_leg.run_leg();
        right_leg.run_leg();

        // update status LED
        status_led.update(data->status);
        #ifdef EXO_DEBUG
            // Serial.println("Exo::Run:Time_OK");
            // Serial.println(delta_t);
            // Serial.println(((float)1 / LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)));
        #endif
        // !!TESTING
        // data->right_leg.ankle.motor.p_des++;

        

        // check for incoming uart messages
        UART_msg_t msg = handler->poll(UART_times::CONT_MCU_TIMEOUT);       //UART_times::CONT_MCU_TIMEOUT is in Config.h
        if (msg.command) {
            // Serial.println("Exo::run->Got message:");
            // UART_msg_t_utils::print_msg(msg);
            UART_command_utils::handle_msg(handler, data, msg);
        }


        // send the coms mcu the real time data every _real_time_msg_delay microseconds
        //Serial.print("Exo::run->Checking if we have to send the message:");
        rt_delta_t += t_helper->tick(rt_context);
        // Serial.print("Exo::run->real_time_del_t: ");Serial.println(rt_delta_t);
        if ((rt_delta_t > BLE_times::_real_time_msg_delay) && (data->status == status_defs::messages::trial_on) || 
        (data->status == status_defs::messages::fsr_calibration) ||
        (data->status == status_defs::messages::fsr_refinement))
        {
            // Serial.println("Exo::run->Sending Real Time Message");
            UART_msg_t msg;
            UART_command_handlers::get_real_time_data(handler, data, msg);
            rt_delta_t = 0;
        }

        delta_t = 0;
    }

    // we didn't hit the time requirements
    else if (delta_t > ((float) 1 / LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)))
    {
        //data->status = status_defs::messages::error;'
        //Serial.println("Exo::Run:Timeoverflow");
        #ifdef EXO_DEBUG
            if (delta_t >= 4000) {
                Serial.println("Exo::Run:Timeoverflow");
                Serial.println(delta_t);
                Serial.println(((float) 1 / LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)));
            }
        #endif
        delta_t = 0;
    }
};



#endif