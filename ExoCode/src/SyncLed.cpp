/*
 * Class to blink and LED in a specific pattern to syncronize the data from the microcontroler with optical tracking systems
 * There will be  _num_start_stop_blinks high pulses at the begining and end of the sequence with each pulse lasting _sync_start_stop_half_period_us.
 * In in between the start and stop it will blink with each high or low lasting _sync_half_period_us.
 * The sequence is started and stopped using trigger().
 * 
 * This setup requires an external timer since the arduino ISR timers can't take in arguments like the class's self.
 * The to change the LED's actual on/off state, update_led must be called.
 * A sample wrapper for the ISR for the timer:
 * 
 *  void grossLedInteruptWrapper(void)
 *  {
 *    syncLed.sync_led_handler(); // calculate the LED state based on the timer, but don't change the actual LED state.
 *    syncTimer.begin(grossLedInteruptWrapper, syncLed.current_sync_period);  // update the timer period ideally we would only do this if it changed, might add a flag to syncLed if needed
 *  }
 * 
 * Then in the main loop or when you are recording data:
 *  int led_state = syncLed.update_led();  // actually change the led state, and record the state in the data
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

#define NUM_START_STOP_BLINKS 1  // the number of times to have the LED on during the start stop sequence.

/*
Constructors
*/

SyncLed::SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us)
{
  // See header file for information on what each variable is for.
	
	
	_pin = pin;
	current_sync_period = sync_half_period_us;
	_sync_start_stop_half_period_us = sync_start_stop_half_period_us;
	_sync_half_period_us = sync_half_period_us;
	_default_led_state = SYNC_LED_ON_STATE;
	led_state = _default_led_state;
    _led_default_state_pin = -1;
    led_is_on = led_state == SYNC_LED_ON_STATE;
  
	_state_change_count = 0; 
	do_blink = false; 
	do_start_stop_sequence = false; 
	_num_start_stop_blinks = NUM_START_STOP_BLINKS;
	
	// Configure the pin for the LED
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin,_default_led_state);

  // Serial for debug.
	// Serial.begin(115200);
	
}

SyncLed::SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us, int default_led_state)
{
	// See header file for information on what each variable is for.
	
	_pin = pin;
	current_sync_period = sync_half_period_us;
	_sync_start_stop_half_period_us = sync_start_stop_half_period_us;
	_sync_half_period_us = sync_half_period_us;
	_default_led_state = default_led_state;
    _led_default_state_pin = -1;
    led_state = _default_led_state;
    led_is_on = led_state == SYNC_LED_ON_STATE;
	
	_state_change_count = 0; // Track how many 
	do_blink = false; // use volatile for shared variables
	do_start_stop_sequence = false; // use volatile for shared variables
	_num_start_stop_blinks = NUM_START_STOP_BLINKS;
	
	// Configure the pin for the LED
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin,_default_led_state);

	// Serial for debug.
	//Serial.begin(115200);
	
}

SyncLed::SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us, int default_led_state , int led_default_state_pin)
{
  // See header file for information on what each variable is for.
  
  _pin = pin;
  current_sync_period = sync_half_period_us;
  _sync_start_stop_half_period_us = sync_start_stop_half_period_us;
  _sync_half_period_us = sync_half_period_us;
  _default_led_state = default_led_state;
  _led_default_state_pin = led_default_state_pin;
  led_state = _default_led_state;
  led_is_on = led_state == SYNC_LED_ON_STATE;
  
  _state_change_count = 0; // Track how many 
  do_blink = false; // use volatile for shared variables
  do_start_stop_sequence = false; // use volatile for shared variables
  _num_start_stop_blinks = NUM_START_STOP_BLINKS;
  
  // Configure the pin for the LED
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin,_default_led_state);

  pinMode(_led_default_state_pin, INPUT_PULLUP);
  _default_led_state = digitalRead(_led_default_state_pin);

  // Serial for debug.
  //Serial.begin(115200);
}

/*
Public
*/


/*
* Sets the flags to start or stop the blink sequence
*/
void SyncLed::trigger()
{
	noInterrupts();  // turn off interupts so we don't accidently call them mid change.  
	do_start_stop_sequence = do_start_stop_sequence ^ true;  // xor with one to change boolean state
	do_blink = do_blink ^ true;  // xor with one to change boolean state
	_state_change_count = 0;  // reset the state change count.
	interrupts();  // turn interupts back on.
}


/*
 * Put the LED in the appropriate state and returns that state for recording
 */
void SyncLed::update_led()
{
	noInterrupts(); // turn off interupts so we don't accidently call them mid change.  
	int temp_led_state = led_state;  // quickly record the state to minimize time without interrupts
	interrupts();// turn interupts back on.
	digitalWrite(_pin, temp_led_state);  // Change the LED state
    _default_led_state = digitalRead(_led_default_state_pin);  // technically this will update for the next call, but functionally this shouldn't really matter as it will change before you can use the sync.
  
    led_is_on = temp_led_state == SYNC_LED_ON_STATE;
	//return led_is_on;
}

/*
 * changes the periods values that are stored.
 */
void SyncLed::update_periods(int sync_start_stop_half_period_us, int sync_half_period_us)
{
	_sync_start_stop_half_period_us = sync_start_stop_half_period_us;
	_sync_half_period_us = sync_half_period_us;
}

/*
 * Calls the approprate methods for setting the LED state based on flags
 */
void SyncLed::sync_led_handler()  // !!! I don't want this to be this long but I think it should be OK if it causes problems with the main interupt I can restructure it.
{
  // do start stop sequence
  if(do_start_stop_sequence){
    _blink_start_stop();
  }
  // do the main blinks
  else if (do_blink){
    _blink();
  }
  // hold default state
  else{
    _default_state();
  }
}

void SyncLed::set_default_state(int new_default)
{
  _default_led_state = new_default;
}


/*
Protected
*/

/*
 * Set values based on start stop sequence
 */
void SyncLed::_blink_start_stop(void)
{
  // Serial.print("blinkStartStop: State Change Count : ");
  // Serial.println(stateChangeCount);

  // If the period is not correct it is the first call in the sequence
  if (current_sync_period == _sync_half_period_us){
    current_sync_period = _sync_start_stop_half_period_us;  // set teh correct period
    // Serial.print("blinkStartStop: sync half period changed to  : ");
    // Serial.println(current_sync_period);
    led_state = SYNC_LED_ON_STATE; // set the LED to on so that way end of the first call in the sequence will be off
  }

  // toggle state
  if (led_state == SYNC_LED_OFF_STATE) {
    led_state = SYNC_LED_ON_STATE;
  } 
  else {
    led_state = SYNC_LED_OFF_STATE;
  }
 
  _state_change_count = _state_change_count + 1;   // iterate the state change counter
  
  // Serial.print("The LED is: ");
  // Serial.println(led_state);

  // once we have done the approprate number of state change stop the start stop sequence.
  if (_state_change_count == (2*_num_start_stop_blinks+1*(_default_led_state==SYNC_LED_ON_STATE))) // if the default state is 1 you need and extra one so make it low before it goes to the default state.
  {
    do_start_stop_sequence = false;
  }
  
  //digitalWrite(syncLEDPin, led_state);
}

/*
 * does the main blink sequence
 */
void SyncLed::_blink(void)
{
  // Serial.print("blinkLED: State Change Count : ");
  // Serial.println(blinkCount);

  // if the period is wrong change it to the correct one.
  if (current_sync_period == _sync_start_stop_half_period_us ){
    current_sync_period = _sync_half_period_us;
    // Serial.print("blinkLED: sync half period changed to  : ");
    // Serial.println(current_sync_period);

  }

  // toggle LED state
  if (led_state == SYNC_LED_OFF_STATE) {
    led_state = SYNC_LED_ON_STATE;
    // _blinkCount = _blinkCount + 1;  // increase when LED turns on
  } 
  else {
    led_state = SYNC_LED_OFF_STATE;
  }
  //digitalWrite(syncLEDPin, led_state);
 
}


/*
 * set the LED to the default value.
 */
void SyncLed :: _default_state()
{
	led_state = _default_led_state;
}
