/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Controller_h
#define Controller_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"

#include "ExoData.h"
#include "Board.h"
#include "ParseIni.h"
#include <stdint.h>
#include "Utilities.h"

//TODO: Create motor base class with interface : int calc_motor_cmd()


/*
 * This class defines the interface for controllers.  
 * All controllers must have a int calc_motor_cmd() that returns a torque cmd in mNm.  
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
		virtual int calc_motor_cmd() = 0;

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
        
        int calc_motor_cmd();
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
        
        int calc_motor_cmd();
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
        
        int calc_motor_cmd();
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
        
        int calc_motor_cmd();
    private:
        void _update_max_angle(float angle);
        void _update_state(float angle);
        void _reset_angles();
        
        // the initial angles used for tracking the extent of the range of motions 
        const float _initial_max_angle = utils::degrees_to_radians(90);
        const float _initial_min_angle = utils::degrees_to_radians(-30);
        
        // Used to track the range of motion
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
        
        int calc_motor_cmd();
    private:
        // when we change a parameter we need to recalculate the splines.
        void _update_spline_parameters(int mass, int peak_normalized_torque_x100, int t0_x10, int t1_x10, int t2_x10, int t3_x10);
                
        // store the parameters so we can check if they change.
        int _mass;
        int _peak_normalized_torque_mNm;
        int _t0_x10;
        int _t1_x10;
        int _t2_x10;
        int _t3_x10;
                
        // peak torque
        float _tp_mNm;
        // cable tension torque.  Not needed for our design, but used to match the paper.
        float _ts_mNm;
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


#endif
#endif