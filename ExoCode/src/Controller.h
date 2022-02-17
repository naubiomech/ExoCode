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
#include "board.h"
#include "parseIni.h"
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


class ProportionalJointMoment : public _Controller
{
    public:
        ProportionalJointMoment(config_defs::joint_id id, ExoData* exo_data);
        ~ProportionalJointMoment(){};
        
        int calc_motor_cmd();
};



class ZeroTorque : public _Controller
{
    public:
        ZeroTorque(config_defs::joint_id id, ExoData* exo_data);
        ~ZeroTorque(){};
        
        int calc_motor_cmd();
};

class HeelToe: public _Controller
{
    public:
        HeelToe(config_defs::joint_id id, ExoData* exo_data);
        ~HeelToe(){};
        
        int calc_motor_cmd();
};


#endif
#endif