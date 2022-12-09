/**
 * @file Controller.h
 *
 * @brief Declares for the different controllers the exo can use. 
 * Controllers should inherit from _Controller class to make sure the interface is the same.
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef Controller_h
#define Controller_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "ExoData.h"
#include "Board.h"
#include "ParseIni.h"
#include <stdint.h>
#include "Utilities.h"
#include "config.h"
#include "Time_Helper.h"
#include <algorithm>

/**
 * @brief This class defines the interface for controllers.  
 * All controllers must have a:
 * float calc_motor_cmd() that returns a torque cmd in Nm.  
 * 
 */
class _Controller
{
	public:
        /**
         * @brief Constructor 
         * 
         * @param id of the joint being used
         * @param pointer to the full ExoData instance
         */
        _Controller(config_defs::joint_id id, ExoData* exo_data);
		
        /**
         * @brief Virtual destructor is needed to make sure the correct destructor is called when the derived class is deleted.
         */
        virtual ~_Controller(){};
		//virtual void set_controller(int joint, int controller) = 0; // Changes the controller for an individual joint
        
        /**
         * @brief Virtual function so that each controller must create a function that will calculate the motor command
         * 
         * @return Torque in Nm.
         */
		virtual float calc_motor_cmd() = 0; 
        
        /**
         * @brief Resets the integral sum for the controller
         */
        void reset_integral(); 
        
    protected:
        
        ExoData* _data; /**< pointer to the full data instance*/
        ControllerData* _controller_data; /**< Pointer to the data associated with this controller */
        LegData* _leg_data; /**< pointer for the leg data the controller is associated with */
        JointData* _joint_data; /**< pointer to the joint data the controller is associated with */
         
        config_defs::joint_id _id;  /**< id of the joint this controller is attached to. */
        
        Time_Helper* _t_helper;  /**< instance of the time helper to track when things happen used to check if we have a set time for the PID */
        float _t_helper_context; /**< store the context for the timer helper */
        float _t_helper_delta_t; /**< time time since the last event */



        // Values for the PID controller
        float _integral_val; /**< sum of the error integral */
        float _prev_input;   /**< prev error term for calculating derivative */
        float _prev_de_dt;   /**< prev error derivative used if the timestep is not good*/
        float _prev_pid_time; /**< prev time the PID was called */
        
        /**
         * @brief calculates the current PID contribution to the motor command. 
         * Currently integral is commented out as resetting _integral_val was crashing the system 
         * 
         * @param controller command 
         * @param measured controlled value
         * @param proportional gain
         * @param integral gain
         * @param derivative gain
         */
        float _pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain);
        
        
}; 

/**
 * @brief Proportional Joint Moment Controller
 * This controller is for the ankle joint 
 * Applies a plantar torque based on the normalized magnitude of the toe FSR
 *
 * 2022-02 : This controller has been around the lab for a while I don't know the original origin -P.Stegall
 *
 * see ControllerData.h for details on the parameters used.
 */
class ProportionalJointMoment : public _Controller
{
    public:
        ProportionalJointMoment(config_defs::joint_id id, ExoData* exo_data);
        ~ProportionalJointMoment(){};
        
        float calc_motor_cmd();
};


/**
 * @brief Zero Torque Controller
 * This controller is for the any joint
 * Simply applies zero torque
 * 
 * see ControllerData.h for details on the parameters used.
 */
class ZeroTorque : public _Controller
{
    public:
        ZeroTorque(config_defs::joint_id id, ExoData* exo_data);
        ~ZeroTorque(){};
        
        float calc_motor_cmd();
};

/**
 * @brief Stasis Controller
 * This controller is for the any joint
 * Simply applies zero torque cmd to motor
 * 
 * see ControllerData.h for details on the parameters used.
 */
class Stasis : public _Controller
{
    public:
        Stasis(config_defs::joint_id id, ExoData* exo_data);
        ~Stasis(){};
        
        float calc_motor_cmd();
};

/**
 * @brief Heel Toe Controller
 * This controller is for the hip joint 
 * Applies torque based on the heel and toe readings with some adjustments for swing
 *
 * 2022-02 : This controller is based on the work of Safoura Sadegh Pour
 * 
 * see ControllerData.h for details on the parameters used.
 */
class HeelToe: public _Controller
{
    public:
        HeelToe(config_defs::joint_id id, ExoData* exo_data);
        ~HeelToe(){};
        
        float calc_motor_cmd();
    
    private:

        float _update_swing_duration();                     /**< Function that updates the average of swing phase duration. */

        static const uint8_t _num_swing_avg = 3;            /**< Number of prior swing phases used to estimate the moving average duration of swing phase. */
        unsigned int _swing_time[_num_swing_avg];           /**< Stores the duration of the last N swing phases. */

        float average_swing_duration;                       /**< Average duration of swing phase from previous N trials. */

        unsigned int _swing_start_timestamp;                /**< Records the time that swing started. */
        unsigned int _heel_strike_timestamp;                /**< Records the time that heel strike occurs. */
        unsigned int _prev_swing_start_timestamp;           /**< Stores the prior time that swing started. */

        float state_2_current;                              /**< Variable to store the current time within "State 2". */
        float state_2_start;                                /**< Variable that stores the time at which "State 2" started. */
        float state_2_end;                                  /**< Variable that stores the time at which "State 2" ended. */ 
        float _prev_state_2;                                /**< Variable to store the previous time duration of "State 2". */
        bool _prev_state_2_status;                          /**< Variable that stores whether or not the prior iteration was in "State 2" [0 = Not "State 2", 1 = "State 2"]. */

        float _fs_previous;                                 /**< Variable that stores the previous fs value. */
        float _df_dt_previous;                              /**< Variable that stores the previous df_dt. */

        float alpha = 1;                                    /**< Shape Tuning Variable 1. */
        float beta = 1;                                     /**< Shape Tuning Variable 2. */

        float swing_duration_window_upper_coeff = 1.75;     /**< factor to multiply by the swing duration to get the upper limit of the window to determine if toe off is considered the start of a new phase of swing. */
        float swing_duration_window_lower_coeff = 0.25;     /**< factor to multiply by the swing duration to get the lower limit of the window to determine if toe off is considered the start of a new phase of swing. */

        float fs;                                           /**< Estimation of ground reaction force based on 'Bishe 2021' */
        float df_dt;                                        /**< Derivative of FS. */

        float cmd_ff;                                        /**< Feedfoward Motor Commnad. */
};

/**
 * @brief Extension Angle Controller
 * This controller is for the hip joint 
 * Applies torque based on the angle of the hip while extending, 
 * then a const torque while flexing
 * 
 * 2022-02 : This controller was conceptualized by Z. Lerner and written by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class ExtensionAngle: public _Controller
{
    public:
        ExtensionAngle(config_defs::joint_id id, ExoData* exo_data);
        ~ExtensionAngle(){};
        
        float calc_motor_cmd();
    private:
       /**
        * @brief calculates the largest angle that the joint has experienced and updates the _max_angle and _min_angle members.
        * 
        * @param current angle for the joint
        */
        void _update_max_angle(float angle);
        
        /**
        * @brief updates the state for if we are doing the hip or the swing phase control
        * 
        * @param current angle for the joint
        */
        void _update_state(float angle);
        
        /**
        * @brief Resets the _max_angle and _min_angle members to the default values. 
        */
        void _reset_angles();
        
        // the initial angles used for tracking the extent of the range of motions 
        const float _initial_max_angle = utils::degrees_to_radians(10); /**< Initial value to set the max angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        const float _initial_min_angle = utils::degrees_to_radians(-5); /**< Initial value to set the min angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        
        // Used to track the range of motion, angles are in rad.
        float _max_angle; /**< The max angle the joint has experienced */
        float _min_angle; /**< The max angle the joint has experienced */
        
        uint8_t _state; /**< Used to track the state 0 is extension mode, 1 is flexion mode. */
      
};

/**
 * @brief BangBang Controller
 * This controller is for the hip joint, but can potentially be applied to other joints 
 * Applies const torque while extending, 
 * then a const torque while flexing
 * 
 * 2022-04 : Based on extension angle controller written by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class BangBang: public _Controller
{
    public:
        BangBang(config_defs::joint_id id, ExoData* exo_data);
        ~BangBang(){};
        
        float calc_motor_cmd();
    private:
        /**
        * @brief calculates the largest angle that the joint has experienced and updates the _max_angle and _min_angle members.
        * 
        * @param current angle for the joint
        */
        void _update_max_angle(float angle);
        
        /**
        * @brief updates the state for if we are doing the hip or the swing phase control
        * 
        * @param current angle for the joint
        */
        void _update_state(float angle);
        
        /**
        * @brief Resets the _max_angle and _min_angle members to the default values. 
        */
        void _reset_angles();
        
        // the initial angles used for tracking the extent of the range of motions 
        const float _initial_max_angle = utils::degrees_to_radians(20); /**< Initial value to set the max angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        const float _initial_min_angle = utils::degrees_to_radians(-5); /**< Initial value to set the min angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        
        // Used to track the range of motion, angles are in rad.
        float _max_angle; /**< The max angle the joint has experienced */
        float _min_angle; /**< The max angle the joint has experienced */
        
        uint8_t _state; /**< Used to track the state 0 is extension mode, 1 is flexion mode. */
      
};

/*
 * LateStance Controller
 * This controller is for the hip joint
 * Applies const torque during late stance
 *
 * see ControllerData.h for details on the parameters used.
 */
class LateStance : public _Controller
{
public:
    LateStance(config_defs::joint_id id, ExoData* exo_data);
    ~LateStance(){};

    float calc_motor_cmd();
private:
    void _update_max_angle(float angle);
    void _update_state(float angle);
    void _reset_angles();

    // the initial angles used for tracking the extent of the range of motions 
    const float _initial_max_angle = utils::degrees_to_radians(20);
    const float _initial_min_angle = utils::degrees_to_radians(-5);

    // Used to track the range of motion, angles are in rad.
    float _max_angle;
    float _min_angle;

    // Used to track the state 0 is extension mode, 1 is flexion mode.
    uint8_t _state;

};

/*
 * GaitPhase Controller
 * This controller is for the hip joint
 * Applies flexion or extension torque as a function of the gait phase (needs heel and toe FSRs)
 *
 * see ControllerData.h for details on the parameters used.
 * 
 * This controller is still in development, this is just meant to be a framework for the controller that will be filled out with time
 */
class GaitPhase : public _Controller
{
    public:
        GaitPhase(config_defs::joint_id id, ExoData* exo_data);
        ~GaitPhase(){};

        float calc_motor_cmd();

        float slope;
        float state;

};

/**
 * @brief Zhang Collins Controller
 * This controller is for the ankle joint 
 * Applies ramp between t0 and t1 to (t1, ts).
 * From t1 to t2 applies a spline going up to (t2,mass*normalized_peak_torque).
 * From t2 to t3 falls to (t3, ts)
 * From t3 to 100% applies zero torque
 * 
 * 2022-02 : This controller was based on:
 * Zhang, J., Fiers, P., Witte, K. A., Jackson, R. W., Poggensee, K. L., Atkeson, C. G., & Collins, S. H. 
 * (2017). Human-in-the-loop optimization of exoskeleton assistance during walking. Science, 356(6344), 1280-1284.
 * and written by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class ZhangCollins: public _Controller
{
    public:
        ZhangCollins(config_defs::joint_id id, ExoData* exo_data);
        ~ZhangCollins(){};
        
        float calc_motor_cmd();
    private:
        /**
         * @brief Recalculates the splines for the curve when the parameters change, and sets the member variables.
         *
         * @param mass of the user in kg
         * @param peak torque normalized by user mass
         * @param percent gait to start ramping up the torque
         * @param percent gait to start the push off spline
         * @param percent gait where peak torque occurs
         * @param percent gait to stop applying torque
         */
        void _update_spline_parameters(int mass, float peak_normalized_torque, float t0, float t1, float t2, float t3);
                
        // store the parameters so we can check if they change.
        int _mass; /**< user mass in kg */
        float _peak_normalized_torque_Nm_kg; /**< peak torque normalized by user mass */
        float _t0; /**< percent gait to start ramping up the torque */
        float _t1; /**< percent gait to start the push off spline */
        float _t2; /**< percent gait where peak torque occurs */
        float _t3; /**< percent gait to stop applying torque */
                
        // peak torque
        float _tp_Nm; /**< Unnormalied peak torque */
        // cable tension torque.  Not needed for our design, but used to match the paper.
        float _ts_Nm; /**< cable tension torque that is ramped up to in early stance.  Not needed for our design, but used to match the paper. */
        // parameters for rising spline
        float _a1; /**< parameter for rising spline */
        float _b1; /**< parameter for rising spline */
        float _c1; /**< parameter for rising spline */
        float _d1; /**< parameter for rising spline */
        
        // parameters for falling spline
        float _a2; /**< parameter for falling spline */
        float _b2; /**< parameter for falling spline */
        float _c2; /**< parameter for falling spline */
        float _d2; /**< parameter for falling spline */
};

/**
 * @brief Franks Collins Controller
 * This controller is for the Hip Joint
 *
 * Is 0 between t0_trough and t1_trough to (t1, 0).
 * From t1_trough to t2_trough applies a spline going down to (t2_trough,mass*normalized_trough_torque).
 * From t2_trough to t3_trough rises to (t3_trough, 0)
 * From t3_trough to t1_peak applies zero torque
 * From t1_peak to t2_peak applies a spline going up to (t2_peak,mass*normalized_peak_torque).
 * From t2_peak to t3_peak falls to (t3_peak, 0)
 *
 * 2022-04 : This controller was based on:
 * Franks, P. W., Bryan, G. M., Martin, R. M., Reyes, R., Lakmazaheri, A. C., & Collins, S. H.
 * (2021). Comparing optimized exoskeleton assistance of the hip, knee, and ankle in single and multi-joint configurations. Wearable Technologies, 2.
 * and written by P. Stegall
 *
 * see ControllerData.h for details on the parameters used.
 */
class FranksCollinsHip: public _Controller
{
    public:
        FranksCollinsHip(config_defs::joint_id id, ExoData* exo_data);
        ~FranksCollinsHip(){};
       
        float calc_motor_cmd();

        float _spline_generation(float node1, float node2, float node3, float torque_magnitude, float shifted_percent_gait);

        float last_percent_gait;
        float last_start_time;
       
};

/**
 * @brief UserDefined Controller
 * This controller uses a simple lookup table with interpolation between values.
 * Assumes table is evenly spaced across percent gait, 0 to 100 where f(0) = f(100)
 * ex. controller_data
 * 
 * 2022-05 : by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
// class UserDefined: public _Controller
// {
    // public:
        // UserDefined(config_defs::joint_id id, ExoData* exo_data);
        // ~UserDefined(){};
        
        // float calc_motor_cmd();
    // private:
        // float _percent_x[controller_defs::user_defined::num_sample_points];
        // const float _step_size = 100/controller_defs::user_defined::num_sample_points;
        
        
// };

/**
 * @brief Sine Controller
 * This controller plays a sine wave with a defined amplitude, period, and phase shift
 * returns amp * sine (frac_of_period * 2 * pi + phase_shift)
 * where frac_of_period is (time % period)/period
 * 
 * 2022-05 : by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class Sine: public _Controller
{
    public:
        Sine(config_defs::joint_id id, ExoData* exo_data);
        ~Sine(){};
        
        float calc_motor_cmd();
    private:
        
        
};

/**
 * @brief Perturbation Controller
 * This controller is meant to work with both the hip and ankle exoskeletons
 * It can apply a torque with a defined amplitude for a user defined duration of time in a user defined direciton (PF/DF)
 *
 * 2022-11 : by J. Williams
 *
 * see ControllerData.h for details on the parameters used.
 */

class Perturbation : public _Controller
{
public:
    Perturbation(config_defs::joint_id id, ExoData* exo_data);
    ~Perturbation() {};

    float calc_motor_cmd();

    float timer;            //Updates with the current time with each pass of the controller
    float start_time;       //Records the time once the controller has been initiated via a button press in the app
    float current_time;     //Calculates the current time relative to the start time
};


#endif
#endif