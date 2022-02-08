/*
 * Takes in the joint id and returns if the left indicator bit is set as a bool
 *
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