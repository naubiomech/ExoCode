/*
 * 
 * 
 * P. Stegall Jan. 2022
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

//TODO: Create motor base class with interface : int calc_motor_cmd()


/*
 * This class defines the interface for controllers.  
 * All controllers must have a float calc_motor_cmd() that returns a torque cmd in Nm.  
 * Torques towards the posterior are positive.
 *
 */
class _Controller
{
	public:
        // Constructor not needed as there isn't anything to set.
        _Controller(config_defs::joint_id id, ExoData* exo_data);
		// Virtual destructor is needed to make sure the correct destructor is called when the derived class is deleted.
        virtual ~_Controller(){};
		//virtual void set_controller(int joint, int controller) = 0; // Changes the controller for an individual joint
		virtual float calc_motor_cmd() = 0;

    protected:
        ExoData* _data;
        ControllerData* _controller_data;
        LegData* _leg_data;
        
        config_defs::joint_id _id;
};

/*
 * Proportional Joint Moment Controller
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


/*
 * Zero Torque Controller
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

/*
 * Heel Toe Controller
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
};

/*
 * Extension Angle Controller
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
 * BangBang Controller
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
 * Zhang Collins Controller
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
        // when we change a parameter we need to recalculate the splines.
        void _update_spline_parameters(int mass, float peak_normalized_torque, float t0, float t1, float t2, float t3);
                
        // store the parameters so we can check if they change.
        int _mass;
        float _peak_normalized_torque_Nm_kg;
        float _t0;
        float _t1;
        float _t2;
        float _t3;
                
        // peak torque
        float _tp_Nm;
        // cable tension torque.  Not needed for our design, but used to match the paper.
        float _ts_Nm;
        // parameters for rising spline
        float _a1;
        float _b1;
        float _c1;
        float _d1;
        
        // parameters for falling spline
        float _a2;
        float _b2;
        float _c2;
        float _d2;
};

/*
 * Franks Collins Controller
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
       
         // when we change a parameter we need to recalculate the splines.
        void _update_spline_parameters(int mass,
            float trough_normalized_torque_Nm_kg, float peak_normalized_torque_Nm_kg,
            float start_percent_gait, float trough_onset_percent_gait, float trough_percent_gait,
            float mid_percent_gait, float mid_duration_gait,
            float peak_percent_gait, float peak_offset_percent_gait);
    private:
        // store the parameters so we can check if they change.
        int _mass;
       
        int _last_start_time;
        float _last_percent_gait;
       
        float _start_percent_gait;
       
        float _mid_time;
        float _mid_duration;
       
        float _trough_normalized_torque_Nm_kg;
        float _peak_normalized_torque_Nm_kg;
       
        float _trough_onset_percent_gait;
        float _trough_percent_gait;
       
        float _peak_percent_gait;
        float _peak_offset_percent_gait;
       
        float _t0_trough;
        float _t1_trough;
        float _t2_trough;
        float _t3_trough;
       
        float _t0_peak;
        float _t1_peak;
        float _t2_peak;
        float _t3_peak;
               
        // peak torque
        float _tt_Nm;
        float _tp_Nm;
       
        // cable tension torque.  Not needed for our design, but used to match the paper.
        float _tts_Nm;
        float _tps_Nm;
       
        // parameters for falling spline
        float _a1_trough;
        float _b1_trough;
        float _c1_trough;
        float _d1_trough;
       
        // parameters for rising spline
        float _a2_trough;
        float _b2_trough;
        float _c2_trough;
        float _d2_trough;
       
        // parameters for rising spline
        float _a1_peak;
        float _b1_peak;
        float _c1_peak;
        float _d1_peak;
       
        // parameters for falling spline
        float _a2_peak;
        float _b2_peak;
        float _c2_peak;
        float _d2_peak;
};

/*
 * UserDefined Controller
 * This controller uses a simple lookup table with interpolation between values.
 * Assumes table is evenly spaced across percent gait, 0 to 100 where f(0) = f(100)
 * ex. controller_data
 * 
 * 2022-05 : by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class UserDefined: public _Controller
{
    public:
        UserDefined(config_defs::joint_id id, ExoData* exo_data);
        ~UserDefined(){};
        
        float calc_motor_cmd();
    private:
        
        
};

/*
 * Sine Controller
 * This controller plays a sine wave with a defined amplitude, period, and phase shift
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


#endif
#endif