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
 * If you need the pin low to be LED on (like with a P channel mosfet) you can change that in the defines below.
 * 
 * P. Stegall Sept. 2021
*/


#ifndef Sync_Led_h
#define Sync_Led_h

#include "Arduino.h"
//#include "IntervalTimer.h"

// Define the on and off state of the LED.  This is handy for if you are using a P Channel MOSFET where low is on.
#define LED_ON_STATE HIGH
#define LED_OFF_STATE LOW

const unsigned int SYNC_HALF_PERIOD_US = 125000;  // half blink period in micro seconds
const unsigned int SYNC_START_STOP_HALF_PERIOD_US = 4 * SYNC_HALF_PERIOD_US; // Half blink period for the begining and end of the sequence.  This is usually longer so it is easy to identify.


// Declare the class
class Sync_Led
{
  public:
    // Constructors one you can set the default LED State
    Sync_Led(int pin, int syncStartStopHalfPeriodUs, int SyncHalfPeriodUs);
  	Sync_Led(int pin, int syncStartStopHalfPeriodUs, int SyncHalfPeriodUs, int defaultLedState);
    
  	void trigger();  // Method call which starts and stops the blink sequence calculation, does not change LED state
  	void updateLed(); // Changes the LED State to the current state
  	void updatePeriods(int syncStartStopHalfPeriodUs, int SyncHalfPeriodUs);  // Used if you need to change the periods after initialization
  	void syncLedHandler();  // Handler which calculates the LED state, and sets the current period the interupt should use.  
    
  	volatile int ledState;  // Records the current LED state. This may not actually be set and depending on the circuit low may be on
    int ledIsOn; // Records if the led is set to on.  This is what you should use to record the value.
	  volatile int currentSyncPeriod; // The current period to use.  Whenever syncLedHandler is called the interupt should have another begin call to make sure the period is correct.
	  volatile bool doBlink ; // flag which says if the blink sequence is active, use volatile for shared variables
    volatile bool doStartStopSequence; // flag for if we are in the start stop region of the sequence, use volatile for shared variables
    
	
    
  private:
  
  	void _blinkStartStop();  // does the start stop blink sequence
  	void _blink(); // does the main blink sequence
  	void _defaultState();  // set LED to default state.
   
  		 
    int _pin;  // pin the LED is on.
  	int _defaultLedState;  // Default LED state
  	
  	int _syncStartStopHalfPeriodUs;  // the time to hold a state, for the start stop sequence
  	int _syncHalfPeriodUs;  // the time to hold a state, for the main blink sequence
  	volatile unsigned int _stateChangeCount ; // store the number of state changes for the start stop sequnce, use volatile for shared variables
  	int _numStartStopBlinks;  // the number of times to have the LED be on during the start stop sequnce,  I think it will always be 1, but there is the option to change it.
	
	
};

#endif
