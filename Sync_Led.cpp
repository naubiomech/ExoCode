/*
 * Class to blink and LED in a specific pattern to syncronize the data from the microcontroler with optical tracking systems
 * There will be  _numStartStopBlinks high pulses at the begining and end of the sequence with each pulse lasting _syncStartStopHalfPeriodUs.
 * In in between the start and stop it will blink with each high or low lasting _syncHalfPeriodUs.
 * The sequence is started and stopped using trigger().
 * 
 * This setup requires an external timer since the arduino ISR timers can't take in arguments like the class's self.
 * The to change the LED's actual on/off state, updateLed must be called.
 * A sample wrapper for the ISR for the timer:
 * 
 *  void grossLedInteruptWrapper(void)
 *  {
 *    syncLed.syncLedHandler(); // calculate the LED state based on the timer, but don't change the actual LED state.
 *    syncTimer.begin(grossLedInteruptWrapper, syncLed.currentSyncPeriod);  // update the timer period ideally we would only do this if it changed, might add a flag to syncLed if needed
 *  }
 * 
 * Then in the main loop or when you are recording data:
 *  int ledState = syncLed.updateLed();  // actually change the led state, and record the state in the data
 *  
 * The static state (not flashing) can be set to either HIGH or LOW
 * 
 * If you need the pin low to be LED on (like with a P channel mosfet) you can change that in the header file defines.
 * 
 * P. Stegall Sept. 2021
*/

#include "Arduino.h"
#include "Sync_Led.h"
//#include <IntervalTimer.h>
//#include "IntervalTimerEx.h"

#define NUM_START_STOP_BLINKS 1  // the number of times to have the LED on during the start stop sequence.

/*
Constructors
*/

Sync_Led::Sync_Led(int pin, int syncStartStopHalfPeriodUs, int syncHalfPeriodUs)
{
  // See header file for information on what each variable is for.
	
	
	_pin = pin;
	currentSyncPeriod = syncHalfPeriodUs;
	_syncStartStopHalfPeriodUs = syncStartStopHalfPeriodUs;
	_syncHalfPeriodUs = syncHalfPeriodUs;
	_defaultLedState = LED_ON_STATE;
	ledState = _defaultLedState;
  _ledDefaultStatePin = -1;
  ledIsOn = ledState == LED_ON_STATE;
  
	_stateChangeCount = 0; 
	doBlink = false; 
	doStartStopSequence = false; 
	_numStartStopBlinks = NUM_START_STOP_BLINKS;
	
	// Configure the pin for the LED
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin,_defaultLedState);

  // Serial for debug.
	// Serial.begin(115200);
	
}

Sync_Led::Sync_Led(int pin, int syncStartStopHalfPeriodUs, int syncHalfPeriodUs, int defaultLedState)
{
	// See header file for information on what each variable is for.
	
	_pin = pin;
	currentSyncPeriod = syncHalfPeriodUs;
	_syncStartStopHalfPeriodUs = syncStartStopHalfPeriodUs;
	_syncHalfPeriodUs = syncHalfPeriodUs;
	_defaultLedState = defaultLedState;
  _ledDefaultStatePin = -1;
  ledState = _defaultLedState;
  ledIsOn = ledState == LED_ON_STATE;
	
	_stateChangeCount = 0; // Track how many 
	doBlink = false; // use volatile for shared variables
	doStartStopSequence = false; // use volatile for shared variables
	_numStartStopBlinks = NUM_START_STOP_BLINKS;
	
	// Configure the pin for the LED
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin,_defaultLedState);

	// Serial for debug.
	//Serial.begin(115200);
	
}

Sync_Led::Sync_Led(int pin, int syncStartStopHalfPeriodUs, int syncHalfPeriodUs, int defaultLedState , int ledDefaultStatePin)
{
  // See header file for information on what each variable is for.
  
  _pin = pin;
  currentSyncPeriod = syncHalfPeriodUs;
  _syncStartStopHalfPeriodUs = syncStartStopHalfPeriodUs;
  _syncHalfPeriodUs = syncHalfPeriodUs;
  _defaultLedState = defaultLedState;
  _ledDefaultStatePin = ledDefaultStatePin;
  ledState = _defaultLedState;
  ledIsOn = ledState == LED_ON_STATE;
  
  _stateChangeCount = 0; // Track how many 
  doBlink = false; // use volatile for shared variables
  doStartStopSequence = false; // use volatile for shared variables
  _numStartStopBlinks = NUM_START_STOP_BLINKS;
  
  // Configure the pin for the LED
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin,_defaultLedState);

  pinMode(_ledDefaultStatePin, INPUT_PULLUP);
  _defaultLedState = digitalRead(_ledDefaultStatePin);

  // Serial for debug.
  //Serial.begin(115200);
}

/*
Public
*/


/*
* Sets the flags to start or stop the blink sequence
*/
void Sync_Led::trigger()
{
	noInterrupts();  // turn off interupts so we don't accidently call them mid change.  
	doStartStopSequence = doStartStopSequence ^ true;  // xor with one to change boolean state
	doBlink = doBlink ^ true;  // xor with one to change boolean state
	_stateChangeCount = 0;  // reset the state change count.
	interrupts();  // turn interupts back on.
}


/*
 * Put the LED in the appropriate state and returns that state for recording
 */
void Sync_Led::updateLed()
{
	noInterrupts(); // turn off interupts so we don't accidently call them mid change.  
	int tempLedState = ledState;  // quickly record the state to minimize time without interrupts
	interrupts();// turn interupts back on.
	digitalWrite(_pin, tempLedState);  // Change the LED state
  _defaultLedState = digitalRead(_ledDefaultStatePin);  // technically this will update for the next call, but functionally this shouldn't really matter as it will change before you can use the sync.
  
  ledIsOn = tempLedState == LED_ON_STATE;
	//return ledIsOn;
}

/*
 * changes the periods values that are stored.
 */
void Sync_Led::updatePeriods(int syncStartStopHalfPeriodUs, int syncHalfPeriodUs)
{
	_syncStartStopHalfPeriodUs = syncStartStopHalfPeriodUs;
	_syncHalfPeriodUs = syncHalfPeriodUs;
}

/*
 * Calls the approprate methods for setting the LED state based on flags
 */
void Sync_Led::syncLedHandler()  // !!! I don't want this to be this long but I think it should be OK if it causes problems with the main interupt I can restructure it.
{
  // do start stop sequence
  if(doStartStopSequence){
    _blinkStartStop();
  }
  // do the main blinks
  else if (doBlink){
    _blink();
  }
  // hold default state
  else{
    _defaultState();
  }
}

void Sync_Led::setDefaultState(int newDefault)
{
  _defaultLedState = newDefault;
}


/*
Protected
*/

/*
 * Set values based on start stop sequence
 */
void Sync_Led::_blinkStartStop(void)
{
  // Serial.print("blinkStartStop: State Change Count : ");
  // Serial.println(stateChangeCount);

  // If the period is not correct it is the first call in the sequence
  if (currentSyncPeriod == _syncHalfPeriodUs){
    currentSyncPeriod = _syncStartStopHalfPeriodUs;  // set teh correct period
    // Serial.print("blinkStartStop: sync half period changed to  : ");
    // Serial.println(currentSyncPeriod);
    ledState = LED_ON_STATE; // set the LED to on so that way end of the first call in the sequence will be off
  }

  // toggle state
  if (ledState == LED_OFF_STATE) {
    ledState = LED_ON_STATE;
  } 
  else {
    ledState = LED_OFF_STATE;
  }
 
  _stateChangeCount = _stateChangeCount + 1;   // iterate the state change counter
  
  // Serial.print("The LED is: ");
  // Serial.println(ledState);

  // once we have done the approprate number of state change stop the start stop sequence.
  if (_stateChangeCount == (2*_numStartStopBlinks+1*(_defaultLedState==LED_ON_STATE))) // if the default state is 1 you need and extra one so make it low before it goes to the default state.
  {
    doStartStopSequence = false;
  }
  
  //digitalWrite(syncLEDPin, ledState);
}

/*
 * does the main blink sequence
 */
void Sync_Led::_blink(void)
{
  // Serial.print("blinkLED: State Change Count : ");
  // Serial.println(blinkCount);

  // if the period is wrong change it to the correct one.
  if (currentSyncPeriod == _syncStartStopHalfPeriodUs ){
    currentSyncPeriod = _syncHalfPeriodUs;
    // Serial.print("blinkLED: sync half period changed to  : ");
    // Serial.println(currentSyncPeriod);

  }

  // toggle LED state
  if (ledState == LED_OFF_STATE) {
    ledState = LED_ON_STATE;
    // _blinkCount = _blinkCount + 1;  // increase when LED turns on
  } 
  else {
    ledState = LED_OFF_STATE;
  }
  //digitalWrite(syncLEDPin, ledState);
 
}


/*
 * set the LED to the default value.
 */
void Sync_Led :: _defaultState()
{
	ledState = _defaultLedState;
}
