#ifndef CALIBRATE_AND_READ_SENSORS_HEADER
#define CALIBRATE_AND_READ_SENSORS_HEADER
// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage

void torque_calibration();

void FSR_calibration();

double get_LL_torq();
double get_RL_torq();

/*FSR Code
	This code is very basic but is kept as an outside function for clarity.
	The FSR readings are used to control state based actions based on the part of the gait cycle the patient is in.
*/
double fsr(const unsigned int pin);
#endif
