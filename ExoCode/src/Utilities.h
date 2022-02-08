#ifndef Utilities_h
#define Utilities_h

#include "parseIni.h"
#include <stdint.h>
/*
 * Takes in the joint id and returns if the left indicator bit is set as a bool
 *
 */
bool get_is_left(config_defs::joint_id id);


/*
 * Takes in the joint id and returns the id with the left/right bits masked out.
 * Returning uint8_t rather than joint_id type since we have to typecast to do logical stuff anyways.
 */
uint8_t get_joint_type(config_defs::joint_id id);






#endif