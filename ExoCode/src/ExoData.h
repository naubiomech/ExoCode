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


//TODO: Add an output data that creates an array that can be used to send to the other board.
// moved status values to StatusLed.h
/* enum class ExoStatus : uint8_t 
{
	not_enabled = 0,
	enabled = 1,
	running = 2,
	error = 3,
}; */

class ExoData 
{
	public:
        ExoData(uint8_t* config_to_send); // constructor
        
        uint8_t status;
        bool sync_led_state;
        LegData left_leg;
        LegData right_leg;
};

#endif