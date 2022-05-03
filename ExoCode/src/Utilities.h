#ifndef Utilities_h
#define Utilities_h

#include "ParseIni.h"
#include "Arduino.h"
#include <stdint.h>
#include <vector>

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

    /*
     * Searches str for 'rmv characters and deletes them all, returns new string
     */
    String remove_all_chars(String str, char rmv);
    
    /*
     * Checks if all elements of the array are equal. Arrays must be the same length and type
     */
    template <typename T>
    int elements_are_equal(T arr1, T arr2, int length)
    {
        for (int i=0; i<length; i++)
        {
            if(arr1[i] != arr2[i])
            {
                return 0;
            }
        }
        return 1;
    };

    /*
     * Sets arr2 elements equal to arr1 elements. Arrays must be the same length and type
     */
    template <typename T>
    void set_elements_equal(T arr1, T arr2, int length)
    {
        for (int i=0; i<length; i++)
        {
            arr1[i] = arr2[i];
        }
    };
    
}


#endif