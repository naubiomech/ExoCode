/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Exo_h
#define Exo_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"

#include "Leg.h"
#include <stdint.h>
#include "ParseIni.h"
#include "Board.h"
#include "Utilities.h"
#include "SyncLed.h"
#include "StatusLed.h"

class Exo
{
    public:
		Exo(ExoData* exo_data); // constructor: uses initializer list for the Leg objects.
		
        void run(); // reads motor data from each motor used in the leg and stores the values
		
        ExoData *data;  // pointer to ExoData that is getting updated by SPI so they share memory.
        
        Leg left_leg;
        Leg right_leg;
        
        SyncLed sync_led;
        StatusLed status_led;
			
	private:
		
};
#endif

#endif