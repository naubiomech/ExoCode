#ifndef MOTOR_HEADER
#define MOTOR_HEADER

class Motor{
private:
	getRawTorque();
public:
	Motor(int motor_pin, int torque_sensor_pin, int err_pin);
	double getTorque();
	bool hasErrored();

	unsigned int torque_sensor_pin;
	unsigned int motor_pin;
	unsigned int err_pin;

	double sign = 1;

	double torque_measurements[dim] = {0};

	double averaged_torque;

	double PID_Setpoint, Input, Output;
	PID pid = PID(&Input, &Output, &PID_Setpoint, PID_DEFAULTS[0], PID_DEFAULTS[1], PID_DEFAULTS[2], DIRECT);

	double ERR;
	double max_KF;
	double min_KF;
	int count_err;

	double KF = 1;

	double torque_calibration_value = 0;
	int Vol;

	int torque_address;

	double Setpoint, Setpoint_Pctrl;
	double Previous_Setpoint = 0;
	double Setpoint_earlyStance = 0.25 * Setpoint;
	double Dorsi_Setpoint;
	double Previous_Dorsi_Setpoint;

	double New_PID_Setpoint = 0.0;
	double Old_PID_Setpoint = 0.0;

	double zero_torque;
};

#endif
