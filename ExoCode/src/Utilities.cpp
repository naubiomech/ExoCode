/*
 * Takes in the joint id and returns if the left indicator bit is set as a bool
 *
 */
#include "Utilities.h"
 
namespace utils
{
    /*
     * From the joint_id returns the bit for is_left.
     */
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
    bool schmitt_trigger(float value, bool is_high, float lower_threshold, float upper_threshold)
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
    
    
    /*
     * The rate limiter is to reduce how quickly a value can change.
     * This is helpful when turning on a controller so the parameter doesn't rapidly change.
     * 
     */
     // Add template so works with ints, floats, whatever.
     int rate_limit(int setpoint, int last_value, int* last_time, int rate_per_ms)
     {
        int time = millis();
        int step = (time - *last_time) * rate_per_ms;
        *last_time = time;
        
        return min(setpoint, last_value + step);
        
        
        
     };
     
    /*
     * sets/clears the specified bit in a unit8_t.
     * Takes in the original uint8_t the bit value you would like to use and the location you are placing that bit.
     */
    uint8_t update_bit(uint8_t original, bool val, uint8_t loc)
    {
        uint8_t keep_bit = ~(1<<loc);  //set a mask for the bits we aren't setting 
        
        return (original & keep_bit) | (val<<loc);
    };
    
    /*
     * Returns the bit in a specific location in a uint8_t
     */
    bool get_bit(uint8_t original, uint8_t loc)
    {
        uint8_t bit_to_check = (1<<loc);  //set a mask for the bits we are checking
        
        return (original & bit_to_check) == bit_to_check;
    };
    
    /*
     * converts from degrees to radians
     */
    float degrees_to_radians(float angle_deg)
    {
        return angle_deg * 2 * PI / 180;
    };
    
    /*
     * converts from radians to degrees
     */
    float radians_to_degrees(float angle_rad)
    {
        return angle_rad * 180 / (2 * PI);
    };

    /*
     * Searches str for 'rmv' characters and deletes them all, returns new string
     */
    String remove_all_chars(String str, char rmv)
    {
        bool found = false;
        while(!found)
        {
            int index = str.indexOf(rmv);
            if (index == -1) 
            {
                found = true;
                continue;
            }
            str.remove(index, 1);
        }
        return str;
    };


    /*
     * given and integer, return the number of characters in it
     */
    int get_char_length(int ofInt)
    {
        int len = 0;
        int localInt = ofInt;
        if (localInt < 0) {
            len += 1;
            //Quick abs(x)
            localInt = ((localInt < 0) ? -1 * localInt : localInt);
        }
        //Faster than loop
        if (localInt < 10) {
            len += 1;
        } else if (localInt < 100) {
            len += 2;
        } else if (localInt < 1000) {
            len += 3;
        } else if (localInt < 10000) {
            len += 4;
        } else if (localInt < 100000) {
            len += 5;
        } else if (localInt < 1000000) {
            len += 6;
        }
        return len;
    };
}