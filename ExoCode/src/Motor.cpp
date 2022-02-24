/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Arduino.h" //TODO: Remove

#include "Motor.h"
#include "CAN.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 


_Motor::_Motor(config_defs::joint_id id, ExoData* exo_data)
{
    _id = id;
    _is_left = ((uint8_t)this->_id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    _data = exo_data;
    
    // set _motor_data to point to the data specific to this motor.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.hip.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.hip.motor);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.knee.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.knee.motor);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.ankle.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.ankle.motor);
            }
            break;
    }
    
};

bool _Motor::get_is_left() // constructor: type is the motor type
{
    return _is_left;
};

config_defs::joint_id _Motor::get_id()
{
    return _id;
};


/*
 * Constructor for the CAN Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the CAN (e.g. TMotor) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model (e.g. AK60), which may have specific torque constants etc.
 * 
 * 
 */
_CANMotor::_CANMotor(config_defs::joint_id id, ExoData* exo_data) // constructor: type is the motor type
: _Motor(id, exo_data)
{
    _KP_MIN = 0.0f;
    _KP_MAX = 500.0f;
    _KD_MIN = 0.0f;
    _KD_MAX = 5.0f;
    _P_MAX = 12.5f;
};

void _CANMotor::read_data()
{
    // read date from motor
    bool searching = true;
    uint8_t data[8];
    uint32_t start = millis();
    CAN* can = can->getInstance();
    do 
    {
        uint32_t id = can->read(data);
        if (id == uint32_t(_motor_data->id))
        {
            // unpack data
            uint32_t p_int = (data[1] << 8) | data[2];
            uint32_t v_int = (data[3]) << 4 | (data[4] >> 4);
            uint32_t i_int = ((data[4] & 0xF) << 8) | data[5];
            // set data in ExoData object
            _motor_data->p = uint_to_float(p_int, -_P_MAX, _P_MAX, 16);
            _motor_data->v = uint_to_float(v_int, -_V_MAX, _V_MAX, 12);
            _motor_data->i = uint_to_float(i_int, -_T_MAX, _T_MAX, 12);
            return;
        }
        searching = ((millis() - start) < _timeout);
    }
    while(searching);
};

void _CANMotor::send_data()
{
    // read data from ExoData object, constraint it, and package it
    float p_sat = constrain(_motor_data->p_des, -_P_MAX, _P_MAX);
    float v_sat = constrain(_motor_data->v_des, -_V_MAX, _V_MAX);
    float kp_sat = constrain(_motor_data->kp, _KP_MIN, _KP_MAX);
    float kd_sat = constrain(_motor_data->kd, _KD_MIN, _KD_MAX);
    float t_sat = constrain(_motor_data->t_ff, -_T_MAX, _T_MAX);
    Serial.print("Sending t_sat:");
    Serial.println(t_sat);
    uint32_t p_int = float_to_uint(p_sat, -_P_MAX, _P_MAX, 16);
    uint32_t v_int = float_to_uint(v_sat, -_V_MAX, _V_MAX, 12);
    uint32_t kp_int = float_to_uint(kp_sat, _KP_MIN, _KP_MAX, 12);
    uint32_t kd_int = float_to_uint(kd_sat, _KD_MIN, _KD_MAX, 12);
    uint32_t t_int = float_to_uint(t_sat, -_T_MAX, _T_MAX, 12);
    uint8_t data[8];
    data[0] = p_int >> 8;
    data[1] = p_int & 0xFF;
    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    data[7] = t_int & 0xFF;
    // set data in motor
    CAN* can = can->getInstance();
    can->send(uint32_t(_motor_data->id), data);
    return;
};

void _CANMotor::on_off(bool is_on)
{
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    if (is_on)
    {
        // enable motor
        data[7] = 0xFC;
    }
    else 
    {
        // disable motor
        data[7] = 0xFD;
    }
    CAN* can = can->getInstance();
    can->send(uint32_t(_motor_data->id), data);
}

float _CANMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12) {
      pgg = (unsigned int) ((x-offset)*4095.0/span); 
    }
    if (bits == 16) {
      pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
}
float _CANMotor::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits == 12) {
      pgg = ((float)x_int)*span/4095.0 + offset;
    }
    if (bits == 16) {
      pgg = ((float)x_int)*span/65535.0 + offset;
    }
    return pgg;
}

//**************************************
/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60::AK60(config_defs::joint_id id, ExoData* exo_data): // constructor: type is the motor type
_CANMotor(id, exo_data)
{
    _T_MAX = 18.0f;
    _V_MAX = 25.65f;
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK80::AK80(config_defs::joint_id id, ExoData* exo_data): // constructor: type is the motor type
_CANMotor(id, exo_data)
{
    _T_MAX = 9.0f;
    _V_MAX = 41.87f;
};


#endif