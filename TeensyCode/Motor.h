/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Motor_h
#define Motor_h



typedef enum {
	AK60 = 1,
	AK80 = 2
} motorType;

struct motorData {
	int id;
	float p; // read position
	float v; // read velocity
	float i; // read current
	float p_des; // 
	float v_des;
	float kp;
	float kd;
	float t_ff;
};

class Motor
{
	public:
		Motor(uint8_t id, motorType motor, bool is_left); // constructor: type is the motor type
		
		void readData(); // reads motor data from each motor used in the leg and stores the values
		void sendData();  // sends new control command to the motors used in the leg, based on the defined controllers
		void setController(int controller); // Changes the low level controller for an individual joint
		void motorOnOff(bool on);  // motor enable/disable
		bool getIsLeft();  // lets you know if it is a left or right leg.
		
		
		
		int id; //motor id
		float p; // read position
		float v; // read velocity
		float i; // read current
		
		float p_des; // 
		float v_des;
		float kp;
		float kd;
		float t_ff;
		
		
	private:
		int _p_int;
		int _v_int;
		int _i_int;
		bool _is_left;
};

#endif