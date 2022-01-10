/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Joint_h
#define Joint_h

#include "Arduino.h"
#include <Motor.h>
#include <Controller.h>
#include <TorqueSensor.h>

struct jointData {
	jointType jointName;
	motorData motor;
	int trqReading;
	controllerData controller;
	struct jointData *nextJoint;
};

typedef enum jointType{
	HIP = 0;
	KNEE = 1;
	ANKLE = 2;
} jointType;

class Joint
{
	public:
		Joint();  // constructor:  
		void runJoint();  // updates the controller and sends the motor command
		void readData(); // reads data from motor and sensors
		void setController(controllerType);  // changes the high level controller in Controller, and the low level controller in Motor
		
		jointType jointName;
		
	private:
		Joint *_nextJoint;
		Motor _motor;
		TorqueSensor _torqueSensor;
		Controller _controller;
};

#endif