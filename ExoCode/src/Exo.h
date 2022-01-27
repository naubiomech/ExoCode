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
#include "parseIni.h"
#include "board.h"


class Exo
{
    public:
		Exo(ExoData* exo_data); // constructor: uses initializer list for the Leg objects.
		
        void run_exo(); // reads motor data from each motor used in the leg and stores the values
		
        ExoData *data;  // pointer to ExoData that is getting updated by SPI so they share memory.
        
        Leg left_leg;
        Leg right_leg;
			
	private:
		
};
#endif

#endif