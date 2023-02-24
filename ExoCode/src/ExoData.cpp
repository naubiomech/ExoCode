#include "ExoData.h"
#include "error_types.h"
#include "Logger.h"

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
    this->error_joint_id = 0;
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
        // logger::print("ExoData::get_joint_with->No joint with ");
        // logger::print(id);
        // logger::println(" was found.");
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
    logger::print("\t Status : ");
    logger::println(_status);
    logger::print("\t Sync LED : ");
    logger::println(sync_led_state);
    
    if (left_leg.is_used)
    {
        logger::print("\tLeft :: FSR Calibration : ");
        logger::print(left_leg.do_calibration_heel_fsr);
        logger::println(left_leg.do_calibration_toe_fsr);
        logger::print("\tLeft :: FSR Refinement : ");
        logger::print(left_leg.do_calibration_refinement_heel_fsr);
        logger::println(left_leg.do_calibration_refinement_toe_fsr);
        logger::print("\tLeft :: Percent Gait : ");
        logger::println(left_leg.percent_gait);
        // logger::print("\tLeft :: Percent Gait Bytes: ");
        // for (unsigned int i = 0; i < sizeof(SPI_DATA_TYPE); i++)
        // {
            // logger::print(_peripheral_message[running_idx_cnt + spi_data_idx::leg::percent_gait-spi_data_idx::leg::idx_cnt+i],HEX);
            // logger::print("\t");
        // }
        // logger::print("\n");
        logger::print("\tLeft :: Heel FSR : ");
        logger::println(left_leg.heel_fsr);
        logger::print("\tLeft :: Toe FSR : ");
        logger::println(left_leg.toe_fsr);
        
        if(left_leg.hip.is_used)
        {
            logger::println("\tLeft :: Hip");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_leg.hip.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_leg.hip.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_leg.hip.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_leg.hip.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_leg.hip.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_leg.hip.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_leg.hip.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_leg.hip.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_leg.hip.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_leg.hip.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_leg.hip.controller.parameter_set);
        }
        
        if(left_leg.knee.is_used)
        {
            logger::println("\tLeft :: Knee");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_leg.knee.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_leg.knee.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_leg.knee.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_leg.knee.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_leg.knee.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_leg.knee.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_leg.knee.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_leg.knee.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_leg.knee.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_leg.knee.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_leg.knee.controller.parameter_set);
        }
        if(left_leg.ankle.is_used)
        {
            logger::println("\tLeft :: Ankle");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_leg.ankle.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_leg.ankle.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_leg.ankle.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_leg.ankle.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_leg.ankle.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_leg.ankle.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_leg.ankle.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_leg.ankle.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_leg.ankle.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_leg.ankle.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_leg.ankle.controller.parameter_set);
        }
    }
    
    if (left_leg.is_used)
    {
        logger::print("\tRight :: FSR Calibration : ");
        logger::print(right_leg.do_calibration_heel_fsr);
        logger::println(right_leg.do_calibration_toe_fsr);
        logger::print("\tRight :: FSR Refinement : ");
        logger::print(right_leg.do_calibration_refinement_heel_fsr);
        logger::println(right_leg.do_calibration_refinement_toe_fsr);
        logger::print("\tRight :: Percent Gait : ");
        logger::println(right_leg.percent_gait);
        logger::print("\tLeft :: Heel FSR : ");
        logger::println(right_leg.heel_fsr);
        logger::print("\tLeft :: Toe FSR : ");
        logger::println(right_leg.toe_fsr);
        
        if(left_leg.hip.is_used)
        {
            logger::println("\tRight :: Hip");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_leg.hip.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_leg.hip.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_leg.hip.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_leg.hip.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_leg.hip.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_leg.hip.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_leg.hip.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_leg.hip.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_leg.hip.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_leg.hip.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_leg.hip.controller.parameter_set);
            
        }
        
        if(left_leg.knee.is_used)
        {
            logger::println("\tRight :: Knee");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_leg.knee.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_leg.knee.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_leg.knee.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_leg.knee.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_leg.knee.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_leg.knee.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_leg.knee.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_leg.knee.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_leg.knee.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_leg.knee.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_leg.knee.controller.parameter_set);
        }
        
        if(left_leg.ankle.is_used)
        {
            logger::println("\tRight :: Ankle");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_leg.ankle.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_leg.ankle.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_leg.ankle.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_leg.ankle.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_leg.ankle.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_leg.ankle.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_leg.ankle.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_leg.ankle.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_leg.ankle.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_leg.ankle.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_leg.ankle.controller.parameter_set);
        }
    }
   
};

