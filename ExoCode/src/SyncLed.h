/*
 * Class to blink and LED in a specific pattern to syncronize the data from the microcontroler with optical tracking systems
 * There will be  _numStartStopBlinks high pulses at the begining and end of the sequence with each pulse lasting _sync_start_stop_half_period_us.
 * In in between the start and stop it will blink with each high or low lasting _sync_half_period_us.
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


#ifndef SyncLed_h
#define SyncLed_h

#include "Arduino.h"
#include "Board.h"
//#include "IntervalTimer.h"

// Define the on and off state of the LED.  This is handy for if you are using a P Channel MOSFET where low is on.
//#define SYNC_LED_ON_STATE HIGH
//#define SYNC_LED_OFF_STATE LOW

const unsigned int SYNC_HALF_PERIOD_US = 125000;  // half blink period in micro seconds
const unsigned int SYNC_START_STOP_HALF_PERIOD_US = 4 * SYNC_HALF_PERIOD_US; // Half blink period for the begining and end of the sequence.  This is usually longer so it is easy to identify.


// Declare the class
class SyncLed
{
  public:
    // Constructors one you can set the default LED State
    SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us);
  	SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us, int default_led_state);
    SyncLed(int pin, int sync_start_stop_half_period_us, int sync_half_period_us, int default_led_state, int led_default_state_pin);  // This isn't a great way of overloading but with high and low being int instead of bool I don't have a great alternative
    
  	void trigger();  // Method call which starts and stops the blink sequence calculation, does not change LED state
  	void update_led(); // Changes the LED State to the current state
  	void update_periods(int sync_start_stop_half_period_us, int sync_half_period_us);  // Used if you need to change the periods after initialization
  	void sync_led_handler();  // Handler which calculates the LED state, and sets the current period the interupt should use.  
    void set_default_state(int new_default);  // Allows you to change the default state on the fly.  Primarily used with digital pin _led_default_state_pin.
    
    
  	volatile int led_state;  // Records the current LED state. This may not actually be set and depending on the circuit low may be on
    int led_is_on; // Records if the led is set to on.  This is what you should use to record the value.
	volatile int current_sync_period; // The current period to use.  Whenever syncLedHandler is called the interupt should have another begin call to make sure the period is correct.
	volatile bool do_blink ; // flag which says if the blink sequence is active, use volatile for shared variables
    volatile bool do_start_stop_sequence; // flag for if we are in the start stop region of the sequence, use volatile for shared variables
    
	
    
  private:
  
  	void _blink_start_stop();  // does the start stop blink sequence
  	void _blink(); // does the main blink sequence
  	void _default_state();  // set LED to default state.
   
  		 
    int _pin;  // pin the LED is on.
  	int _default_led_state;  // Default LED state
    int _led_default_state_pin;  // Pin for external switch for setting LED state -1 if not used.
  	
  	int _sync_start_stop_half_period_us;  // the time to hold a state, for the start stop sequence
  	int _sync_half_period_us;  // the time to hold a state, for the main blink sequence
  	volatile unsigned int _state_change_count ; // store the number of state changes for the start stop sequnce, use volatile for shared variables
  	unsigned int _num_start_stop_blinks;  // the number of times to have the LED be on during the start stop sequnce,  I think it will always be 1, but there is the option to change it.
	
	
};

#endif
