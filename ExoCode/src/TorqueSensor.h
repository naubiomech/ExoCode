/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef TorqueSensor_h
#define TorqueSensor_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "board.h"
#include "Arduino.h"

struct torqueData {
	int rawReading;
	int calibratedReading;
};


class TorqueSensor
{
	public:
		TorqueSensor(unsigned int pin);
        void calibrate(); // Changes the controller for an individual joint
		int read(); // reads the pins and updatas the data stuct.
				
		
	private:
		bool _is_used;
        int _pin;
        int _calibration;
        int _rawReading;
		int _calibratedReading;
        
};
#endif
#endif