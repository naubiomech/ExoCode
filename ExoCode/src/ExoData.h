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
#include "parseIni.h"
#include "board.h"


enum class ExoStatus {
	not_enabled = 0,
	enabled = 1,
	running = 2,
	error = 3,
};

class ExoData 
{
	public:
        ExoData(uint8_t* config_to_send); // constructor
        
        ExoStatus status;
        bool sync_led_state;
        LegData left_leg;
        LegData right_leg;
};

#endif