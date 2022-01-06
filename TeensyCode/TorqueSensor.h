/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Controller_h
#define Controller_h

struct torqueData {
	int rawReading;
	int calibratedReading;
};

class TorqueSensor
{
	public:
		TorqueSensor(int pin);
		void calibrate(); // Changes the controller for an individual joint
		int read(); // reads the pins and updatas the data stuct.
				
		
	private:
		int _rawReading;
		int _calibratedReading;
};

#endif