/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Joint.h"


// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
// initialize the used joint counters that will be used to select the TorqueSensor pin.  If you don't do it it won't work.
uint8_t _Joint::left_used_joint_count = 0;
uint8_t _Joint::right_used_joint_count = 0;

/*
 * Constructor for the joint
 * Takes the joint id and a pointer to the exo_data
 * Uses initializer list for motor, controller, and torque sensor.
 * Only stores these objects, the id, exo_data pointer, and if it is left (for easy access)
 */
_Joint::_Joint(config_defs::joint_id id, ExoData* exo_data)
: _torque_sensor(_Joint::get_torque_sensor_pin(id, exo_data))
//, _controller(id, exo_data)
{
    _id = id;
    _is_left = utils::get_is_left(_id); //((uint8_t)this->id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
        
    _data = exo_data;

    // Serial.print(uint8_t(id));
    // Serial.print("\t");
    // Serial.print(_is_used);
    // Serial.print("\t");
    // Serial.print(uint8_t(_torque_sensor._pin));
    // Serial.print("\t");
    
    // set _joint_data to point to the data specific to this joint.
    // switch (utils::get_joint_type(_id))
    // {
        // case (uint8_t)config_defs::joint_id::hip:
            // if (_is_left)
            // {
                // _joint_data = &(exo_data->left_leg.hip);
            // }
            // else
            // {
                // _joint_data = &(exo_data->right_leg.hip);
            // }
            // break;
            
        // case (uint8_t)config_defs::joint_id::knee:
            // if (_is_left)
            // {
                // _joint_data = &(exo_data->left_leg.knee);
            // }
            // else
            // {
                // _joint_data = &(exo_data->right_leg.knee);
            // }
            // break;
        
        // case (uint8_t)config_defs::joint_id::ankle:
            // if (_is_left)
            // {
                // _joint_data = &(exo_data->left_leg.ankle);
            // }
            // else
            // {
                // _joint_data = &(exo_data->right_leg.ankle);
            // }
            // break;
    // }
};  

/*
 * reads data for sensors for the joint, torque and motor.
 */
void _Joint::read_data()  
{
    // Read the torque sensor, and change sign based on side.
    _joint_data->torque_reading = _joint_data->flip_direction ? -1*_torque_sensor.read() : _torque_sensor.read();
    
    _joint_data->position = _joint_data->motor.p / _joint_data->motor.gearing;
    _joint_data->velocity = _joint_data->motor.v / _joint_data->motor.gearing;
};

/*
 * Checks if we need to do the calibration for the motor and sensors
 * and runs the calibration.
 */
void _Joint::check_calibration()  
{
    // Serial.print("id: ");
    // Serial.print(uint8_t(_id));
    // Serial.print("\t");
    // Check if we are doing the calibration on the torque sensor
    _joint_data->calibrate_torque_sensor = _torque_sensor.calibrate(_joint_data->calibrate_torque_sensor);
    //Serial.print("_Joint::check_calibration\n"); 
};

/*
 * Takes in the joint id and exo data, and checks if the current joint is used.
 * If it is used it pulls the next open torque sensor pin for the side, and increments the counter.
 * If the joint is not used, or we have used up all the available torque sensor pins for the side, it sets the pin to a pin that is not connected.
 */
unsigned int _Joint::get_torque_sensor_pin(config_defs::joint_id id, ExoData* exo_data)
{
    // First check which joint we are looking at.  
    // Then go through and if it is the left or right and if it is used.  
    // If it is set return the appropriate pin and increment the counter.
    switch (utils::get_joint_type(id))
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            if (utils::get_is_left(id) & exo_data->left_leg.hip.is_used)  // check if the left leg is used
            {
                if (_Joint::left_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_left[_Joint::left_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else if (exo_data->right_leg.hip.is_used)  // check if the right leg is used
            {
                if (_Joint::right_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_right[_Joint::right_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else  // the joint isn't used.  I didn't optimize for the minimal number of logical checks because this should just be used at startup.
            {
                return logic_micro_pins::not_connected_pin;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            if (utils::get_is_left(id) & exo_data->left_leg.knee.is_used)  // check if the left leg is used
            {
                if (_Joint::left_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_left[_Joint::left_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else if (exo_data->right_leg.knee.is_used)  // check if the right leg is used
            {
                if (_Joint::right_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_right[_Joint::right_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else  // the joint isn't used.  I didn't optimize for the minimal number of logical checks because this should just be used at startup.
            {
                return logic_micro_pins::not_connected_pin;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            if (utils::get_is_left(id) & exo_data->left_leg.ankle.is_used)  // check if the left leg is used
            {
                if (_Joint::left_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_left[_Joint::left_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else if (exo_data->right_leg.ankle.is_used)  // check if the right leg is used
            {
                if (_Joint::right_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_right[_Joint::right_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else  // the joint isn't used.  I didn't optimize for the minimal number of logical checks because this should just be used at startup.
            {
                return logic_micro_pins::not_connected_pin;
            }
            break;
        }
        default :
        {
            return logic_micro_pins::not_connected_pin;
        }

    }
};

/*
 * A method to set the _motor to point to the new motor.  
 * Not fully needed since we are only setting motors at the beginning but used for consistent formatting.
 */
void _Joint::set_motor(_Motor* new_motor)
{
    _motor = new_motor;
};


//*********************************************
HipJoint::HipJoint(config_defs::joint_id id, ExoData* exo_data)
: _Joint(id, exo_data)
, _zero_torque(id, exo_data)
, _heel_toe(id, exo_data)
, _extension_angle(id, exo_data)
{

    //Serial.print("HipJoint::HipJoint\n");
    // set _joint_data to point to the data specific to this joint.
    if (_is_left)
    {
        _joint_data = &(exo_data->left_leg.hip);
    }
    else
    {
        _joint_data = &(exo_data->right_leg.hip);
    }
    // Serial.print(uint8_t(id));
    // Serial.print("\t");
    // Serial.print(_joint_data->is_used);
    // Serial.print("\t");
    // Don't need to check side as we assume symmetry and create both leg data objects.
    // setup motor from here as it will be easier to check which motor is used
    if(_joint_data->is_used)
    {
        switch (exo_data->left_leg.hip.motor.motor_type)
        {
            // using new so the object of the specific motor type persists.
            case (uint8_t)config_defs::motor::AK60 :
                //_motor = new AK60(id, exo_data);
                HipJoint::set_motor(new AK60(id, exo_data));
                break;
            case (uint8_t)config_defs::motor::AK80 :
                //_motor = new AK80(id, exo_data);
                HipJoint::set_motor(new AK80(id, exo_data));
                break;
            default :
                //_motor = nullptr;
                HipJoint::set_motor(new NullMotor(id, exo_data));
                break;
        }
        delay(5);
        
        set_controller(exo_data->left_leg.hip.controller.controller);
    }
};

/*
 * Calculates and sends motor commands, all data should be read prior to runing this.
 */
void HipJoint::run_joint()
{
    // Calculate the motor command
    _joint_data->controller.setpoint = _controller->calc_motor_cmd();
    // Send the new command to the motor.
    // Serial.print(_joint_data->controller.setpoint);
    // Serial.print(" Hip\t");
    // Use transaction because the motors are call and response
    _motor->transaction(0 / _joint_data->motor.gearing);
    // _motor->transaction(_joint_data->controller.setpoint / _joint_data->motor.gearing);
};  

/*
 * reads data for sensors for the joint, torque and motor.
 */
 /*
void HipJoint::read_data() 
{
    // Check if we are doing the calibration on the torque sensor
    calibrate_torque_sensor = _torque_sensor.calibrate(calibrate_torque_sensor);
    
    // Read the torque sensor.
    _torque_sensor.read();
};
*/

/*
 * Changes the high level controller in Controller
 * Each joint has their own version since they have joint specific controllers.
 */
void HipJoint::set_controller(uint8_t controller_id)   
{
    switch (controller_id)
    {
        case (uint8_t)config_defs::hip_controllers::disabled :
            _motor->on_off(false);
            _controller = &_zero_torque;
            break;
        case (uint8_t)config_defs::hip_controllers::zero_torque :
            _controller = &_zero_torque;
            break;
        case (uint8_t)config_defs::hip_controllers::heel_toe :
            _controller = &_heel_toe;
            break;
        case (uint8_t)config_defs::hip_controllers::extension_angle :
            _controller = &_extension_angle;
            break;
        default :
            _controller = nullptr;
            break;
    } 
};

//================================================================

KneeJoint::KneeJoint(config_defs::joint_id id, ExoData* exo_data)
: _Joint(id, exo_data)
, _zero_torque(id, exo_data)
{
    // Serial.print("KneeJoint::KneeJoint\n");
    // set _joint_data to point to the data specific to this joint.
    if (_is_left)
    {
        _joint_data = &(exo_data->left_leg.knee);
    }
    else
    {
        _joint_data = &(exo_data->right_leg.knee);
    }
    // Serial.print(uint8_t(id));
    // Serial.print("\t");
    // Serial.print(_joint_data->is_used);
    // Serial.print("\t");
    // Don't need to check side as we assume symmetry and create both leg data objects.
    // setup motor from here as it will be easier to check which motor is used
   if(_joint_data->is_used)
   {
        switch (_data->left_leg.knee.motor.motor_type)
        {
            // using new so the object of the specific motor type persists.
            case (uint8_t)config_defs::motor::AK60 :
                KneeJoint::set_motor(new AK60(id, exo_data));
                break;
            case (uint8_t)config_defs::motor::AK80 :
                KneeJoint::set_motor(new AK80(id, exo_data));
                break;
            default :
                KneeJoint::set_motor(new NullMotor(id, exo_data));
                break;
        }
        delay(5);

        set_controller(exo_data->left_leg.knee.controller.controller);
   }
};

/*
 * Reads the data and sends motor commands
 */
void KneeJoint::run_joint()
{
    // Calculate the motor command
    _joint_data->controller.setpoint = _controller->calc_motor_cmd();
    // Send the new command to the motor.
    // Serial.print(_joint_data->controller.setpoint);
    // Serial.print(" Knee\t");
    // Use transaction because the motors are call and response
    _motor->transaction(0 / _joint_data->motor.gearing);
};  

/*
 * reads data for sensors for the joint, torque and motor.
 */
 /*
void KneeJoint::read_data() // reads data from motor and sensors
{
    // Check if we are doing the calibration on the torque sensor
    calibrate_torque_sensor = _torque_sensor.calibrate(calibrate_torque_sensor);
    
    // Read the torque sensor.
    _torque_sensor.read();
};
*/

/*
 * Changes the high level controller in Controller
 * Each joint has their own version since they have joint specific controllers.
 */
void KneeJoint::set_controller(uint8_t controller_id)  // changes the high level controller in Controller, and the low level controller in Motor
{
    switch (controller_id)
    {
        case (uint8_t)config_defs::knee_controllers::disabled :
            _motor->on_off(false);
            _controller = &_zero_torque;
            break;
        case (uint8_t)config_defs::knee_controllers::zero_torque :
            _controller = &_zero_torque;
            break;
        default :
            _controller = nullptr;
            break;
    } 
};

//=================================================================
AnkleJoint::AnkleJoint(config_defs::joint_id id, ExoData* exo_data)
: _Joint(id, exo_data)
, _zero_torque(id, exo_data)
, _proportional_joint_moment(id, exo_data)
, _zhang_collins(id, exo_data)
{
    // Serial.print("AnkleJoint::AnkleJoint\n");
    // set _joint_data to point to the data specific to this joint.
    if (_is_left)
    {
        _joint_data = &(exo_data->left_leg.ankle);
    }
    else
    {
        _joint_data = &(exo_data->right_leg.ankle);
    }
    // Serial.print(uint8_t(id));
    // Serial.print("\t");
    // Serial.print(_joint_data->is_used);
    // Serial.print("\t");
            
    // Don't need to check side as we assume symmetry and create both leg data objects.
    // setup motor from here as it will be easier to check which motor is used
    if(_joint_data->is_used)
    {
        switch (_data->left_leg.ankle.motor.motor_type)
        {
            // using new so the object of the specific motor type persists.
            case (uint8_t)config_defs::motor::AK60 :
                AnkleJoint::set_motor(new AK60(id, exo_data));
                break;
            case (uint8_t)config_defs::motor::AK80 :
                AnkleJoint::set_motor(new AK80(id, exo_data));
                break;
            default :
            AnkleJoint::set_motor(new NullMotor(id, exo_data));
                break;
        }
        delay(5);

        set_controller(exo_data->left_leg.ankle.controller.controller);
    }  
};

/*
 * Reads the data and sends motor commands
 */
void AnkleJoint::run_joint()
{
    // Calculate the motor command
    _joint_data->controller.setpoint = _controller->calc_motor_cmd();
    // Send the new command to the motor.
    // Serial.print(_joint_data->controller.setpoint);
    // Serial.print("\t");
    // Use transaction because the motors are call and response
    _motor->transaction(0 / _joint_data->motor.gearing);
};  

/*
 * reads data for sensors for the joint, torque and motor.
 */
 /*
void AnkleJoint::read_data()  
{
    // Check if we are doing the calibration on the torque sensor
    calibrate_torque_sensor = _torque_sensor.calibrate(calibrate_torque_sensor);
    
    // Read the torque sensor.
    _torque_sensor.read();
};
*/

/*
 * Changes the high level controller in Controller
 * Each joint has their own version since they have joint specific controllers.
 */
void AnkleJoint::set_controller(uint8_t controller_id)  // changes the high level controller in Controller, and the low level controller in Motor
{
    switch (controller_id)
    {
        case (uint8_t)config_defs::ankle_controllers::disabled :
            _motor->on_off(false);
            _controller = &_zero_torque;
            break;
        case (uint8_t)config_defs::ankle_controllers::zero_torque :
            _controller = &_zero_torque;
            break;
        case (uint8_t)config_defs::ankle_controllers::pjmc :
            _controller = &_proportional_joint_moment;
            break;
        case (uint8_t)config_defs::ankle_controllers::zhang_collins :
            _controller = &_zhang_collins;
            break;
        default :
            _controller = nullptr;
            break;
    } 
};
#endif