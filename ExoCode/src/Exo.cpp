/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Exo.h"
#include "Time_Helper.h"

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
, sync_led(logic_micro_pins::sync_led_pin, sync_time::SYNC_START_STOP_HALF_PERIOD_US, sync_time::SYNC_HALF_PERIOD_US, logic_micro_pins::sync_led_on_state, logic_micro_pins::sync_default_pin)  // Create a sync LED object, the first and last arguments (pin) are found in Board.h, and the rest are in Sync_Led.h.  If you do not have a digital input for the default state you can remove SYNC_DEFAULT_STATE_PIN.  
, status_led(logic_micro_pins::status_led_r_pin, logic_micro_pins::status_led_g_pin, logic_micro_pins::status_led_b_pin)  // Create the status LED object. 
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
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static float context = t_helper->generate_new_context();
    static float delta_t = 0;
    delta_t += t_helper->tick(context);
    
    if (((delta_t <= ((float) 1/LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE))) && (delta_t >= ((float)1 / LOOP_FREQ_HZ * 1000000 * (1 - LOOP_TIME_TOLERANCE)))))
    {

        // check if anything new has come in over SPI

        // check if we should update the sync LED and record the LED on/off state.
        data->sync_led_state = sync_led.handler();
        bool trial_running = sync_led.get_is_blinking();

        // check the estop
        data->estop = digitalRead(logic_micro_pins::motor_stop_pin);

        // Serial.print("Exo::run: is error : ");
        // Serial.print(((data->status & status_defs::messages::error) == status_defs::messages::error));
        // Serial.print("\n");
        if (trial_running && (((data->status & status_defs::messages::error) != status_defs::messages::error) && (data->status != status_defs::messages::test)))
        {
            data->status = status_defs::messages::trial_on;
            // Serial.print("Exo::run:trial on\n");
        }
        else if ((!trial_running) && (((data->status & status_defs::messages::error) != status_defs::messages::error) && (data->status != status_defs::messages::test)))
        {
            data->status = status_defs::messages::trial_off;
            // Serial.print("Exo::run:trial off\n");
        }
        else
        {
            // Serial.print("Exo::run:Error or Test\n");
        }

        // Record the leg data and send new commands to the motors.
        left_leg.run_leg();
        right_leg.run_leg();

        // update status LED
        status_led.update(data->status);

        // Serial.println("Exo::Run:Time_OK");
        // Serial.println(delta_t);
        // Serial.println(((float)1 / LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)));

        delta_t = 0;
        // send data over SPI
    }
    else if (delta_t > ((float) 1 / LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)))
    {
        //data->status = status_defs::messages::error;
        if (delta_t >= 4000) {
            Serial.println("Exo::Run:Timeoverflow");
            Serial.println(delta_t);
            Serial.println(((float) 1 / LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)));
        }
        delta_t = 0;
    }
};

#endif