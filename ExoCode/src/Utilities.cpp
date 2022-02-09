/*
 * Takes in the joint id and returns if the left indicator bit is set as a bool
 *
 */
#include "Utilities.h"
 
namespace utils
{
    bool get_is_left(config_defs::joint_id id)
    {
        return ((uint8_t)id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    };

    /*
     * Takes in the joint id and returns the id with the left/right bits masked out.
     * Returning uint8_t rather than joint_id type since we have to typecast to do logical stuff anyways.
     */
    uint8_t get_joint_type(config_defs::joint_id id)
    {
        return (uint8_t)id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right);  // return the joint id with the left/right indicators masked out.  
    };
    
    /*
     * A schmitt trigger is a way of tracking if a noisy signal is high or low 
     * When it is low it must go above the upper threshold before it is high
     * When it is high it must go below the lower threshold before it is low.
     * This way if the signal crosses one threshold multiple times it won't register as changing multiple times.
     * 
     * Takes in the value, current state, the lower threshold, and upper threshold
     * Returns the updated state
     */
    bool schmitt_trigger(int value, bool is_high, int lower_threshold, int upper_threshold)
    {
        bool trigger = 0;
        if (is_high)
        {
            trigger = value > lower_threshold;
        }
        else
        {
            trigger = value > upper_threshold;
        }    
        return trigger;
    }
}