/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Motor_h
#define Motor_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "ExoData.h"
#include "ParseIni.h"
#include "Board.h"
#include "Utilities.h"


#include <stdint.h>

// TODO: Create base motor class with interface read_data(), send_cmd(), motor_on_off(bool), get_is_left()

class _Motor
{
	public:
		_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_Motor(){};
		
        //Pure virtual functions, these will have to be defined for each one.
        virtual void read_data() = 0; // reads motor data from each motor used in the leg and stores the values
		virtual void send_data(float torque) = 0;  // sends new control command to the motors used in the leg, based on the defined controllers
		virtual void transaction(float torque) = 0;
        //void set_controller(int controller); // Changes the low level controller for an individual joint
		virtual bool on_off(bool is_on) = 0;  // motor enable/disable
        virtual bool on_off(bool is_on, bool overide) = 0;  // motor enable/disable
        virtual void zero() = 0; // set position to zero
		 
        virtual bool get_is_left();  // lets you know if it is a left or right leg.
        virtual config_defs::joint_id get_id();
		
		MotorData* _motor_data;
	protected:
        config_defs::joint_id _id; //motor id 
		bool _is_left;
        ExoData* _data;
        int _enable_pin;
        bool _prev_motor_enabled;
};

class NullMotor : public _Motor
{
    public:
    NullMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin):_Motor(id, exo_data, enable_pin) {};
    void read_data() {};
    void send_data(float torque) {};
    void transaction(float torque) {};
    bool on_off(bool is_on) {return true;};
    bool on_off(bool is_on, bool overide) {return true;};
    void zero() {};
};


/*
 * This will define some of the common communication 
 */
class _CANMotor : public _Motor
{
    public:
        _CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_CANMotor(){};
        void transaction(float torque);
        void read_data();
        void send_data(float torque);
        bool on_off(bool is_on);
        bool on_off(bool is_on, bool overide);
        void zero();
    protected:
        float _float_to_uint(float x, float x_min, float x_max, int bits);
        float _uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
        void _handle_read_failure();
        float _KP_MIN;
        float _KP_MAX;
        float _KD_MIN;
        float _KD_MAX;
        float _P_MAX;
        float _T_MAX;
        float _V_MAX;
        int _timeout_count = 0;
        bool _powered;
        bool _enable_response;
        const uint32_t _timeout = 500; //micro-seconds
};


class AK60 : public _CANMotor
{
    public:
        AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // constructor: type is the motor type
		~AK60(){};
};

class AK60_v1_1 : public _CANMotor
{
    public:
        AK60_v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // constructor: type is the motor type
		~AK60_v1_1(){};
};

class AK80 : public _CANMotor
{
    public:
        AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // constructor: type is the motor type
		~AK80(){};   
};

#endif
#endif