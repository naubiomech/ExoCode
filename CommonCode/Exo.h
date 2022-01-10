/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Exo_h
#define Exo_h

#include <Leg.h>

struct exoData {
	legData leg;
};

typedef enum exoStatus {
	notEnabled = 0;
	enabled = 1
	running = 2;
	error = 3;	
} exoStatus

class Exo
{
	public:
		Exo(!!calibrationData!!); // constructor: uses all joints.
			
		void runExo(); // reads motor data from each motor used in the leg and stores the values
		
		exoData data;
		exoStatus status;
		
	private:
		
};

#endif