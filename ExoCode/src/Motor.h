/**
 * @file Motor.h
 *
 * @brief Declares a class used to interface with motors
 * 
 * @author P. Stegall 
 * @date Jan. 2022
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

/**
 * @brief Abstract class to define the interface for all motors.
 * All controllers must have a:
 * void read_data()
 * void send_data(float torque)
 * void transaction(float torque)
 * void on_off()
 * bool enable()
 * bool enable(bool overide)
 * void zero()
 * bool get_is_left()
 * config_defs::joint_id get_id()
 */
class _Motor
{
	public:
		_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_Motor(){};
		
        //Pure virtual functions, these will have to be defined for each one.
        
        /**
         * @brief reads motor data from each motor used in the leg and stores the values
         */
        virtual void read_data() = 0; 

        /**
         * @brief Sends the new motor command to the motor.
         * 
         * @param motor torque command in Nm
         */
		virtual void send_data(float torque) = 0;  
		
        /**
         * @brief Sends the new motor command to the motor and reads the current state of the motor.
         * 
         * @param motor torque command in Nm
         */
        virtual void transaction(float torque) = 0;
        //void set_controller(int controller); // Changes the low level controller for an individual joint
		
        /**
         * @brief Powers on or off the motors depending on the is_on value in motor data 
         */
        virtual void on_off() = 0;  
        
        /**
         * @brief Enables or disables the motors depending on the state stored in the corresponding enabled state in motor data.
         * Only sends commands if the state has changes in the motor data.
         */
        virtual bool enable() = 0;  
        
        /**
         * @brief same as enable but will resend commands if override is true, regardless of what the state of the system is.
         */
        virtual bool enable(bool overide) = 0;  
        
        /**
         * @brief set position to zero
         */
        virtual void zero() = 0; 
		 
        
        /**
         * @brief lets you know if it is a left or right leg.
         * 
         * @return 1 if the motor is on the left side, 0 otherwise
         */
        virtual bool get_is_left();  // 
        
        /**
         * @brief returns the motor id, same as the joint id
         *
         * @return the motor id
         */
        virtual config_defs::joint_id get_id();
		
	protected:
        config_defs::joint_id _id; //motor id 
		bool _is_left;
        ExoData* _data;
		MotorData* _motor_data;
        int _enable_pin;
        bool _prev_motor_enabled; 
        bool _prev_on_state;
};

/**
 * @brief a motor that does nothing
 */
class NullMotor : public _Motor
{
    public:
    NullMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin):_Motor(id, exo_data, enable_pin) {};
    void read_data() {};
    void send_data(float torque) {};
    void transaction(float torque) {};
    void on_off() {};
    bool enable() {return true;};
    bool enable(bool overide) {return true;};
    void zero() {};
};


/**
 * @brief This will define some of the common communication used by all the CAN motors and should be inherited by all of them.
 */
class _CANMotor : public _Motor
{
    public:
        _CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_CANMotor(){};
        void transaction(float torque);
        void read_data();
        void send_data(float torque);
        void on_off();
        bool enable();
        bool enable(bool overide);
        void zero();
    protected:
        
        /**
         * @brief Packs a float into the uint format needed to be sent to the motor.
         *
         * @param Float to be packed
         * @param Lower limit of the range of x values, used for scaling
         * @param Upper limit of the range of x values, used for scaling
         * @param Number of bits to pack the value into, 12 or 16
         *
         * @return Should return a uint that has been scaled to a position between x_min and x_max.  Currently returns a float, but it seems to work.
         */
        //TODO Chance : change the return to uint and check that stuff still works
        float _float_to_uint(float x, float x_min, float x_max, int bits);
        
        /**
         * @brief Unpacks a unsigned int format from the motor into a float.
         *
         * @param Unsigned int to be unpacked
         * @param Lower limit of the range of x values, used for scaling
         * @param Upper limit of the range of x values, used for scaling
         * @param Number of bits to pack the value into, 12 or 16
         *
         * @return unpacked float value 
         */
        float _uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
        
        /**
         * @brief Detects timeouts in case of a read failure.
         *
         */
        void _handle_read_failure();
        
        float _KP_MIN; /**< Lower limit of the P gain for the motor */
        float _KP_MAX; /**< Upper limit of the P gain for the motor */
        float _KD_MIN; /**< Lower limit of the D gain for the motor */
        float _KD_MAX; /**< Upper limit of the D gain for the motor */
        float _P_MAX; /**< Max angle of the motor */
        float _T_MAX; /**< Max torque of the motor */
        float _V_MAX; /**< Max velocity of the motor */
        int _timeout_count = 0; /**< Current count of the number of timeouts */
        bool _enable_response; /**< True if the motor responded to an enable command */
        const uint32_t _timeout = 500;  /**< Time to wait for response from the motor in micro-seconds */
};

/**
 * @brief Class for AK60 V1.0 motor
 */
class AK60 : public _CANMotor
{
    public:
        AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // constructor: type is the motor type
		~AK60(){};
};

/**
 * @brief Class for AK60 V1.1 motor
 */
class AK60_v1_1 : public _CANMotor
{
    public:
        AK60_v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // constructor: type is the motor type
		~AK60_v1_1(){};
};

/**
 * @brief Class for AK80 V1.0 motor
 */
class AK80 : public _CANMotor
{
    public:
        AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // constructor: type is the motor type
		~AK80(){};   
};

#endif
#endif