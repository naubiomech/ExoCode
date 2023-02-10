#include "ExoData.h"
#include "error_types.h"

/*
 * Constructor for the exo data.
 * Takes the array from the INI parser.
 * Stores the exo status, and the sync LED state.
 * TODO: decide if we want to change the sync LED state to a pointer. Or bring the whole sync LED object into the exo.
 * Uses an initializer list for the leg data. 
 */
ExoData::ExoData(uint8_t* config_to_send) 
: left_leg(true, config_to_send)  // using initializer list for member objects.
, right_leg(false, config_to_send)
{
    this->_status = status_defs::messages::trial_off;
    this->sync_led_state = false;
    this->estop = false;

    this->config = config_to_send;
    this->config_len = ini_config::number_of_keys;

    this->mark = 10;  

    this->error_code = NO_ERROR;
    this->user_paused = false;
};

void ExoData::reconfigure(uint8_t* config_to_send) 
{
    left_leg.reconfigure(config_to_send);
    right_leg.reconfigure(config_to_send);
};

void ExoData::for_each_joint(for_each_joint_function_t function, float* args)
{
    function(&left_leg.hip, args);
    function(&left_leg.knee, args);
    function(&left_leg.ankle, args);
    function(&right_leg.hip, args);
    function(&right_leg.knee, args);
    function(&right_leg.ankle, args);
};

void ExoData::for_each_joint(for_each_joint_function_t function)
{
    function(&left_leg.hip, NULL);
    function(&left_leg.knee, NULL);
    function(&left_leg.ankle, NULL);
    function(&right_leg.hip, NULL);
    function(&right_leg.knee, NULL);
    function(&right_leg.ankle, NULL);
};

uint8_t ExoData::get_used_joints(uint8_t* used_joints)
{
    uint8_t len = 0;

    used_joints[len] = ((left_leg.hip.is_used) ? (1) : (0));
    len += left_leg.hip.is_used;
    used_joints[len] = ((left_leg.knee.is_used) ? (1) : (0));
    len += left_leg.knee.is_used;
    used_joints[len] = ((left_leg.ankle.is_used) ? (1) : (0));
    len += left_leg.ankle.is_used;
    used_joints[len] = ((right_leg.hip.is_used) ? (1) : (0));
    len += right_leg.hip.is_used;
    used_joints[len] = ((right_leg.knee.is_used) ? (1) : (0));
    len += right_leg.knee.is_used;
    used_joints[len] = ((right_leg.ankle.is_used) ? (1) : (0));
    len += right_leg.ankle.is_used;
    return len;
};

JointData* ExoData::get_joint_with(uint8_t id)
{
    JointData* j_data = NULL;
    switch (id)
    {
    case (uint8_t)config_defs::joint_id::left_hip:
        j_data = &left_leg.hip;
        break;
    case (uint8_t)config_defs::joint_id::left_knee:
        j_data = &left_leg.knee;
        break;
    case (uint8_t)config_defs::joint_id::left_ankle:
        j_data = &left_leg.ankle;
        break;
    case (uint8_t)config_defs::joint_id::right_hip:
        j_data = &right_leg.hip;
        break;
    case (uint8_t)config_defs::joint_id::right_knee:
        j_data = &right_leg.knee;
        break;
    case (uint8_t)config_defs::joint_id::right_ankle:
        j_data = &right_leg.ankle;
        break; 
    default:
        // Serial.print("ExoData::get_joint_with->No joint with ");
        // Serial.print(id);
        // Serial.println(" was found.");
        break;
    }
    return j_data;
};

void ExoData::set_status(uint16_t status_to_set)
{
    // If the status is already error, don't change it
    if (this->_status == status_defs::messages::error)
    {
        return;
    }
    this->_status = status_to_set;
}

uint16_t ExoData::get_status(void)
{
    return this->_status;
};


void ExoData::print()
{
    Serial.print("\t Status : ");
    Serial.println(_status);
    Serial.print("\t Sync LED : ");
    Serial.println(sync_led_state);
    
    if (left_leg.is_used)
    {
        Serial.print("\tLeft :: FSR Calibration : ");
        Serial.print(left_leg.do_calibration_heel_fsr);
        Serial.println(left_leg.do_calibration_toe_fsr);
        Serial.print("\tLeft :: FSR Refinement : ");
        Serial.print(left_leg.do_calibration_refinement_heel_fsr);
        Serial.println(left_leg.do_calibration_refinement_toe_fsr);
        Serial.print("\tLeft :: Percent Gait : ");
        Serial.println(left_leg.percent_gait);
        // Serial.print("\tLeft :: Percent Gait Bytes: ");
        // for (unsigned int i = 0; i < sizeof(SPI_DATA_TYPE); i++)
        // {
            // Serial.print(_peripheral_message[running_idx_cnt + spi_data_idx::leg::percent_gait-spi_data_idx::leg::idx_cnt+i],HEX);
            // Serial.print("\t");
        // }
        // Serial.print("\n");
        Serial.print("\tLeft :: Heel FSR : ");
        Serial.println(left_leg.heel_fsr);
        Serial.print("\tLeft :: Toe FSR : ");
        Serial.println(left_leg.toe_fsr);
        
        if(left_leg.hip.is_used)
        {
            Serial.println("\tLeft :: Hip");
            Serial.print("\t\tcalibrate_torque_sensor : ");
            Serial.println(left_leg.hip.calibrate_torque_sensor);
            Serial.print("\t\ttorque_reading : ");
            Serial.println(left_leg.hip.torque_reading);
            Serial.print("\t\tMotor :: p : ");
            Serial.println(left_leg.hip.motor.p);
            Serial.print("\t\tMotor :: v : ");
            Serial.println(left_leg.hip.motor.v);
            Serial.print("\t\tMotor :: i : ");
            Serial.println(left_leg.hip.motor.i);
            Serial.print("\t\tMotor :: p_des : ");
            Serial.println(left_leg.hip.motor.p_des);
            Serial.print("\t\tMotor :: v_des : ");
            Serial.println(left_leg.hip.motor.v_des);
            Serial.print("\t\tMotor :: t_ff : ");
            Serial.println(left_leg.hip.motor.t_ff);
            Serial.print("\t\tController :: controller : ");
            Serial.println(left_leg.hip.controller.controller);
            Serial.print("\t\tController :: setpoint : ");
            Serial.println(left_leg.hip.controller.setpoint);
            Serial.print("\t\tController :: parameter_set : ");
            Serial.println(left_leg.hip.controller.parameter_set);
        }
        
        if(left_leg.knee.is_used)
        {
            Serial.println("\tLeft :: Knee");
            Serial.print("\t\tcalibrate_torque_sensor : ");
            Serial.println(left_leg.knee.calibrate_torque_sensor);
            Serial.print("\t\ttorque_reading : ");
            Serial.println(left_leg.knee.torque_reading);
            Serial.print("\t\tMotor :: p : ");
            Serial.println(left_leg.knee.motor.p);
            Serial.print("\t\tMotor :: v : ");
            Serial.println(left_leg.knee.motor.v);
            Serial.print("\t\tMotor :: i : ");
            Serial.println(left_leg.knee.motor.i);
            Serial.print("\t\tMotor :: p_des : ");
            Serial.println(left_leg.knee.motor.p_des);
            Serial.print("\t\tMotor :: v_des : ");
            Serial.println(left_leg.knee.motor.v_des);
            Serial.print("\t\tMotor :: t_ff : ");
            Serial.println(left_leg.knee.motor.t_ff);
            Serial.print("\t\tController :: controller : ");
            Serial.println(left_leg.knee.controller.controller);
            Serial.print("\t\tController :: setpoint : ");
            Serial.println(left_leg.knee.controller.setpoint);
            Serial.print("\t\tController :: parameter_set : ");
            Serial.println(left_leg.knee.controller.parameter_set);
        }
        if(left_leg.ankle.is_used)
        {
            Serial.println("\tLeft :: Ankle");
            Serial.print("\t\tcalibrate_torque_sensor : ");
            Serial.println(left_leg.ankle.calibrate_torque_sensor);
            Serial.print("\t\ttorque_reading : ");
            Serial.println(left_leg.ankle.torque_reading);
            Serial.print("\t\tMotor :: p : ");
            Serial.println(left_leg.ankle.motor.p);
            Serial.print("\t\tMotor :: v : ");
            Serial.println(left_leg.ankle.motor.v);
            Serial.print("\t\tMotor :: i : ");
            Serial.println(left_leg.ankle.motor.i);
            Serial.print("\t\tMotor :: p_des : ");
            Serial.println(left_leg.ankle.motor.p_des);
            Serial.print("\t\tMotor :: v_des : ");
            Serial.println(left_leg.ankle.motor.v_des);
            Serial.print("\t\tMotor :: t_ff : ");
            Serial.println(left_leg.ankle.motor.t_ff);
            Serial.print("\t\tController :: controller : ");
            Serial.println(left_leg.ankle.controller.controller);
            Serial.print("\t\tController :: setpoint : ");
            Serial.println(left_leg.ankle.controller.setpoint);
            Serial.print("\t\tController :: parameter_set : ");
            Serial.println(left_leg.ankle.controller.parameter_set);
        }
    }
    
    if (left_leg.is_used)
    {
        Serial.print("\tRight :: FSR Calibration : ");
        Serial.print(right_leg.do_calibration_heel_fsr);
        Serial.println(right_leg.do_calibration_toe_fsr);
        Serial.print("\tRight :: FSR Refinement : ");
        Serial.print(right_leg.do_calibration_refinement_heel_fsr);
        Serial.println(right_leg.do_calibration_refinement_toe_fsr);
        Serial.print("\tRight :: Percent Gait : ");
        Serial.println(right_leg.percent_gait);
        Serial.print("\tLeft :: Heel FSR : ");
        Serial.println(right_leg.heel_fsr);
        Serial.print("\tLeft :: Toe FSR : ");
        Serial.println(right_leg.toe_fsr);
        
        if(left_leg.hip.is_used)
        {
            Serial.println("\tRight :: Hip");
            Serial.print("\t\tcalibrate_torque_sensor : ");
            Serial.println(right_leg.hip.calibrate_torque_sensor);
            Serial.print("\t\ttorque_reading : ");
            Serial.println(right_leg.hip.torque_reading);
            Serial.print("\t\tMotor :: p : ");
            Serial.println(right_leg.hip.motor.p);
            Serial.print("\t\tMotor :: v : ");
            Serial.println(right_leg.hip.motor.v);
            Serial.print("\t\tMotor :: i : ");
            Serial.println(right_leg.hip.motor.i);
            Serial.print("\t\tMotor :: p_des : ");
            Serial.println(right_leg.hip.motor.p_des);
            Serial.print("\t\tMotor :: v_des : ");
            Serial.println(right_leg.hip.motor.v_des);
            Serial.print("\t\tMotor :: t_ff : ");
            Serial.println(right_leg.hip.motor.t_ff);
            Serial.print("\t\tController :: controller : ");
            Serial.println(right_leg.hip.controller.controller);
            Serial.print("\t\tController :: setpoint : ");
            Serial.println(right_leg.hip.controller.setpoint);
            Serial.print("\t\tController :: parameter_set : ");
            Serial.println(right_leg.hip.controller.parameter_set);
            
        }
        
        if(left_leg.knee.is_used)
        {
            Serial.println("\tRight :: Knee");
            Serial.print("\t\tcalibrate_torque_sensor : ");
            Serial.println(right_leg.knee.calibrate_torque_sensor);
            Serial.print("\t\ttorque_reading : ");
            Serial.println(right_leg.knee.torque_reading);
            Serial.print("\t\tMotor :: p : ");
            Serial.println(right_leg.knee.motor.p);
            Serial.print("\t\tMotor :: v : ");
            Serial.println(right_leg.knee.motor.v);
            Serial.print("\t\tMotor :: i : ");
            Serial.println(right_leg.knee.motor.i);
            Serial.print("\t\tMotor :: p_des : ");
            Serial.println(right_leg.knee.motor.p_des);
            Serial.print("\t\tMotor :: v_des : ");
            Serial.println(right_leg.knee.motor.v_des);
            Serial.print("\t\tMotor :: t_ff : ");
            Serial.println(right_leg.knee.motor.t_ff);
            Serial.print("\t\tController :: controller : ");
            Serial.println(right_leg.knee.controller.controller);
            Serial.print("\t\tController :: setpoint : ");
            Serial.println(right_leg.knee.controller.setpoint);
            Serial.print("\t\tController :: parameter_set : ");
            Serial.println(right_leg.knee.controller.parameter_set);
        }
        
        if(left_leg.ankle.is_used)
        {
            Serial.println("\tRight :: Ankle");
            Serial.print("\t\tcalibrate_torque_sensor : ");
            Serial.println(right_leg.ankle.calibrate_torque_sensor);
            Serial.print("\t\ttorque_reading : ");
            Serial.println(right_leg.ankle.torque_reading);
            Serial.print("\t\tMotor :: p : ");
            Serial.println(right_leg.ankle.motor.p);
            Serial.print("\t\tMotor :: v : ");
            Serial.println(right_leg.ankle.motor.v);
            Serial.print("\t\tMotor :: i : ");
            Serial.println(right_leg.ankle.motor.i);
            Serial.print("\t\tMotor :: p_des : ");
            Serial.println(right_leg.ankle.motor.p_des);
            Serial.print("\t\tMotor :: v_des : ");
            Serial.println(right_leg.ankle.motor.v_des);
            Serial.print("\t\tMotor :: t_ff : ");
            Serial.println(right_leg.ankle.motor.t_ff);
            Serial.print("\t\tController :: controller : ");
            Serial.println(right_leg.ankle.controller.controller);
            Serial.print("\t\tController :: setpoint : ");
            Serial.println(right_leg.ankle.controller.setpoint);
            Serial.print("\t\tController :: parameter_set : ");
            Serial.println(right_leg.ankle.controller.parameter_set);
        }
    }
   
};

