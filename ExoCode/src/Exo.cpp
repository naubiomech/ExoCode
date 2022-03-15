/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Exo.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the Exo
 * Takes the exo_data
 * Uses initializer list for legs.
 * Only stores these objects, and exo_data pointer.
 */
Exo::Exo(ExoData* exo_data)
: left_leg(true, exo_data)
, right_leg(false, exo_data) // constructor: uses initializer list for the legs.
, sync_led(logic_micro_pins::sync_led_pin, SYNC_START_STOP_HALF_PERIOD_US, SYNC_HALF_PERIOD_US, logic_micro_pins::sync_led_on_state, logic_micro_pins::sync_default_pin)  // Create a sync LED object, the first and last arguments (pin) are found in Board.h, and the rest are in Sync_Led.h.  If you do not have a digital input for the default state you can remove SYNC_DEFAULT_STATE_PIN.  
, status_led(logic_micro_pins::status_led_r_pin, logic_micro_pins::status_led_g_pin, logic_micro_pins::status_led_b_pin)  // Create the status LED object. 
{
    this->data = exo_data; 
};

/* 
 * Run the exo 
 */
void Exo::run()
{
    // check if anything new has come in over SPI
    
    // check if we should update the sync LED and record the LED on/off state.
    data->sync_led_state = sync_led.handler();
    bool trial_running = sync_led.get_is_blinking();
    if (trial_running & (((data->status & status_led_defs::messages::error) != status_led_defs::messages::error) & (data->status != status_led_defs::messages::test)))
    {
        data->status = status_led_defs::messages::trial_on;
    }
    else if (!trial_running & (((data->status & status_led_defs::messages::error) != status_led_defs::messages::error) & (data->status != status_led_defs::messages::test)))
    {
        data->status = status_led_defs::messages::trial_off;
    }
    
    // Record the leg data and send new commands to the motors.
    left_leg.run_leg();
    right_leg.run_leg();
    
    // update status LED
    status_led.update(data->status);
    
    
    // send data over SPI
};

#endif