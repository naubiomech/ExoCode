#ifndef Utilities_h
#define Utilities_h

#include "parseIni.h"
#include <stdint.h>

namespace utils
{
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
    
    /*
     * Takes in the current value, if the state is currently high, and the lower and upper threshold
     * Returns the new state of the system based on the value and past state.
     * Floats are used so that ints will be promoted but it will still work with floats.  May cause issues with very large ints.
     * Templates could be used in the future but seemed to have issues if all types were not present in the template, e.g. is_high is always a bool.
     */
    bool schmitt_trigger(float value, bool is_high, float lower_threshold, float upper_threshold);
    
    /*
     * Limits the rate at which a value can change.  This is useful when you would like a variable to gradually come on.
     */
    int rate_limit(int setpoint, int last_value, int* last_time, int rate_per_ms);
    
    /*
     * sets/clears the specified bit in a unit8_t.
     * Takes in the original uint8_t the bit value you would like to use and the location you are placing that bit.
     */
    uint8_t update_bit(uint8_t original, bool val, uint8_t loc);
    
    /*
     * Returns the bit in a specific location in a uint8_t
     */
    bool get_bit(uint8_t original, uint8_t loc);
    
    /*
     * converts from degrees to radians
     */
    float degrees_to_radians(float);
    
    /*
     * converts from radians to degrees
     */
    float radians_to_degrees(float);
    
    
}






#endif