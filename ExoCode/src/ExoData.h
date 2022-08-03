/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef ExoData_h
#define ExoData_h

#include "Arduino.h"

#include "LegData.h"
#include <stdint.h>
#include "ParseIni.h"
#include "Board.h"
#include "StatusLed.h"

/* 
 * ExoData was broken out from the Exo class as we were originally going to have it mirrored on a second microcontroller that handled BLE.
 * It doesn't need to be done this way if we aren't, and is pretty cumbersome.
 * Just thought you might be wondering about the weirdness.
 */

//TODO: Add an output data that creates an array that can be used to send to the other board.
// moved status values to StatusLed.h
/* enum class ExoStatus : uint8_t 
{
	not_enabled = 0,
	enabled = 1,
	running = 2,
	error = 3,
}; */

// Type used for the for each joint method, the function should take JointData as input and return void
typedef void (*for_each_joint_function_t) (JointData*);

class ExoData 
{
	public:
        ExoData(uint8_t* config_to_send); // constructor
        void reconfigure(uint8_t* config_to_send);
        void for_each_joint(for_each_joint_function_t function);
        
        uint8_t status;
        bool sync_led_state;
        bool estop;
        LegData left_leg;
        LegData right_leg;
};

#endif