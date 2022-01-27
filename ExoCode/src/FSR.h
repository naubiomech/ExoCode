/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef FSR_h
#define FSR_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"
#include "board.h"


class FSR
{
	public:
		FSR(int pin);
		void calibrate(); // Changes the controller for an individual joint
		int read(); // reads the pins and updatas the data stuct.
		
			
		
	private:
		int _calibration;
		int _rawReading;
		int _calibratedReading;
        int _pin;
};
#endif
#endif