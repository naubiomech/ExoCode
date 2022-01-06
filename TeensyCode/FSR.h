/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef FSR_h
#define FSR_h

#include "Arduino.h"

struct FSRData {
	int calibration;
	int rawReading;
	int calibratedReading;
};

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
};

#endif