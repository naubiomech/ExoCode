/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Leg_h
#define Leg_h

#include "Arduino.h"

#include <motor.h>
#include <controller.h>
#include <FSR.h>



struct legData{
	jointData joint;
	int percentGait; // likely want to do fixed point 
	int heelFSR;
	int toeFSR;
	bool isLeft;
	struct legData *nextLeg;
};

class Leg
{
	public:
		Leg(); // constructor: 
		
		void runLeg(); // read FSR,  calc percent gait, read joint data, send joint commands
		
		void readData(); // reads motor data from each motor used in the leg and stores the values
		void updateMotorCmds();  // sends new control command to the motors used in the leg, based on the defined controllers
		void setController(int joint, int controller); // Changes the controller for an individual joint
		
		
		
	private:
		void _calcPercentGait();
		jointStruct _joints;
		int _percentGait;
		FSR _heelFSR;
		FSR _toeFSR;
		bool _isLeft; // flag to track which side the leg
		Leg *_nextLeg;
		
		
};

#endif