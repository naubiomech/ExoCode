/*
 * Class to blink and LED in a specific pattern to syncronize the data from the microcontroler with optical tracking systems
 * There will be  _num_start_stop_blinks high pulses at the begining and end of the sequence with each pulse lasting _sync_start_stop_half_period_us.
 * In in between the start and stop it will blink with each high or low lasting _sync_half_period_us.
 * The sequence is started and stopped using trigger().
 * 
 * This implementation after initializing the object, you call trigger() to start or stop the pattern.
 * handler() should be called every loop to turn the LED on or off as appropriate and to record if the LED is on
 * get_should_stream() can be called to determine if the rest of the data should be streaming.
 *
 * 
 * Then in the main loop or when you are recording data:
 *  int _led_state = syncLed.update_led();  // actually change the led state, and record the state in the data
 *  
 * The static state (not flashing) can be set to either HIGH or LOW
 * 
 * If you need the pin low to be LED on (like with a P channel mosfet) you can change that in the header file defines.
 * 
 * P. Stegall Sept. 2021
*/

#include "Arduino.h"
#include "SyncLed.h"
//#include <IntervalTimer.h>
//#include "IntervalTimerEx.h"
// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#define NUM_START_STOP_BLINKS 1  // the number of times to have the LED on during the start stop sequence.

/*
Constructors
*/

SyncLed::SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us)
{
  // See header file for information on what each variable is for.
	
	
	_pin = pin;
	_current_sync_period = sync_half_period_us;
	_sync_start_stop_half_period_us = sync_start_stop_half_period_us;
    _last_state_change_timestamp_us = 0;
	_sync_half_period_us = sync_half_period_us;
	_default_led_state = logic_micro_pins::sync_led_on_state;
	_led_state = _default_led_state;
    _led_default_state_pin = -1;
    _led_is_on = _led_state == logic_micro_pins::sync_led_on_state;
    
	_state_change_count = 0; 
	_do_blink = false; 
	_do_start_stop_sequence = false; 
	_num_start_stop_blinks = NUM_START_STOP_BLINKS;
	_is_blinking = false;
    
	// Configure the pin for the LED
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin,_default_led_state);

  // Serial for debug.
	// Serial.begin(115200);
	
};

SyncLed::SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us, int default_led_state)
{
	// See header file for information on what each variable is for.
	
	_pin = pin;
	_current_sync_period = sync_half_period_us;
	_sync_start_stop_half_period_us = sync_start_stop_half_period_us;
    _last_state_change_timestamp_us = 0;
	_sync_half_period_us = sync_half_period_us;
	_default_led_state = default_led_state;
    _led_default_state_pin = -1;
    _led_state = _default_led_state;
    _led_is_on = _led_state == logic_micro_pins::sync_led_on_state;
	
	_state_change_count = 0; // Track how many 
	_do_blink = false; // use volatile for shared variables
	_do_start_stop_sequence = false; // use volatile for shared variables
	_num_start_stop_blinks = NUM_START_STOP_BLINKS;
	_is_blinking = false;
    
	// Configure the pin for the LED
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin,_default_led_state);

	// Serial for debug.
	//Serial.begin(115200);
	
};

SyncLed::SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us, int default_led_state , int led_default_state_pin)
{
  // See header file for information on what each variable is for.
  
  _pin = pin;
  _current_sync_period = sync_half_period_us;
  _sync_start_stop_half_period_us = sync_start_stop_half_period_us;
  _last_state_change_timestamp_us = 0;
  _sync_half_period_us = sync_half_period_us;
  _default_led_state = default_led_state;
  _led_default_state_pin = led_default_state_pin;
  _led_state = _default_led_state;
  _led_is_on = _led_state == logic_micro_pins::sync_led_on_state;
  
  _state_change_count = 0; // Track how many 
  _do_blink = false; // use volatile for shared variables
  _do_start_stop_sequence = false; // use volatile for shared variables
  _num_start_stop_blinks = NUM_START_STOP_BLINKS;
  _is_blinking = false;
  
  // Configure the pin for the LED
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin,_default_led_state);

  pinMode(_led_default_state_pin, INPUT_PULLUP);
  _default_led_state = digitalRead(_led_default_state_pin);

  // Serial for debug.
  //Serial.begin(115200);
};

/*
Public
*/


/*
* Sets the flags to start or stop the blink sequence
*/
void SyncLed::trigger()
{
	_do_start_stop_sequence = _do_start_stop_sequence ^ true;  // xor with one to change boolean state
	_do_blink = _do_blink ^ true;  // xor with one to change boolean state
	_state_change_count = 0;  // reset the state change count.
};


/*
 * Put the LED in the appropriate state and returns that state for recording
 */
void SyncLed::update_led()
{
	int temp_led_state = _led_state;  // quickly record the state to minimize time without interrupts
	digitalWrite(_pin, temp_led_state);  // Change the LED state
    _default_led_state = digitalRead(_led_default_state_pin);  // technically this will update for the next call, but functionally this shouldn't really matter as it will change before you can use the sync.
  
    _led_is_on = temp_led_state == logic_micro_pins::sync_led_on_state;
	//return led_is_on;
};

/*
 * changes the periods values that are stored.
 */
void SyncLed::update_periods(int sync_start_stop_half_period_us, int sync_half_period_us)
{
	_sync_start_stop_half_period_us = sync_start_stop_half_period_us;
	_sync_half_period_us = sync_half_period_us;
};

/*
 * Calls the appropriate methods for setting the LED state based on flags
 */
bool SyncLed::handler()  
{
    int timestamp_us = micros();
    if ((timestamp_us - _last_state_change_timestamp_us) >= _current_sync_period)
    {
        _last_state_change_timestamp_us = timestamp_us;
        // do start stop sequence
        if(_do_start_stop_sequence)
        {
            _blink_start_stop();
            _is_blinking = true;  // this way it will be recorded as blinking anytime not in the default state
        }
        // do the main blinks
        else if (_do_blink)
        {
            _blink();
        }
        // hold default state
        else
        {
            _is_blinking = false; // in the default state it is not blinking.
            _default_state();
        }
      
        update_led();
    }
  return _led_is_on;
  
};

void SyncLed::set_default_state(int new_default)
{
  _default_led_state = new_default;
};


/*
Protected
*/

/*
 * Set values based on start stop sequence. This is a separate section to allow for use with interrupts.
 */
void SyncLed::_blink_start_stop(void)
{
  // Serial.print("blinkStartStop: State Change Count : ");
  // Serial.print(stateChangeCount);
  //Serial.print("\n");

  // If the period is not correct it is the first call in the sequence
  if (_current_sync_period == _sync_half_period_us){
    _current_sync_period = _sync_start_stop_half_period_us;  // set the correct period
    // Serial.print("blinkStartStop: sync half period changed to  : ");
    // Serial.print(current_sync_period);
    // Serial.print("\n");
    _led_state = logic_micro_pins::sync_led_on_state; // set the LED to on so that way end of the first call in the sequence will be off
  }

  // toggle state
  if (_led_state == logic_micro_pins::sync_led_off_state) {
    _led_state = logic_micro_pins::sync_led_on_state;
  } 
  else {
    _led_state = logic_micro_pins::sync_led_off_state;
  }
 
  _state_change_count = _state_change_count + 1;   // iterate the state change counter
  
  // Serial.print("The LED is: ");
  // Serial.print(_led_state);
  // Serial.print("\n");

  // once we have done the appropriate number of state change stop the start stop sequence.
  if (_state_change_count == (2*_num_start_stop_blinks+1*(_default_led_state==logic_micro_pins::sync_led_on_state))) // if the default state is 1 you need and extra one so make it low before it goes to the default state.
  {
    _do_start_stop_sequence = false;
  }
  
  //digitalWrite(syncLEDPin, _led_state);
};

/*
 * does the main blink sequence. This is a separate section to allow for use with interrupts.
 */
void SyncLed::_blink(void)
{
  // Serial.print("blinkLED: State Change Count : ");
  // Serial.print(blinkCount);
  // Serial.print("\n");

  // if the period is wrong change it to the correct one.
  if (_current_sync_period == _sync_start_stop_half_period_us ){
    _current_sync_period = _sync_half_period_us;
    // Serial.print("blinkLED: sync half period changed to  : ");
    // Serial.print(current_sync_period);
    // Serial.print("\n");
  }

  // toggle LED state
  if (_led_state == logic_micro_pins::sync_led_off_state) {
    _led_state = logic_micro_pins::sync_led_on_state;
    // _blinkCount = _blinkCount + 1;  // increase when LED turns on
  } 
  else {
    _led_state = logic_micro_pins::sync_led_off_state;
  }
  //digitalWrite(syncLEDPin, _led_state);
};


/*
 * set the LED to the default value.
 */
void SyncLed::_default_state()
{
	_led_state = _default_led_state;
};

/*
 * returns if the led is on or off.
 */
bool SyncLed::get_led_is_on()
{
    return _led_is_on;
}; 

/*
 * returns if the led is on or off.
 */
bool SyncLed::get_is_blinking()
{
    return _is_blinking;
}; 

#endif