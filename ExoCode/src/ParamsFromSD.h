/*
 * 
 * 
 * P. Stegall July 2022
*/


#ifndef ParamsFromSD_h
#define ParamsFromSD_h

#include "ExoData.h"
#include "ParseIni.h"
#include "Utilities.h"

#include <SD.h>
#include <SPI.h>
#include <map>
#include <string>


// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    #ifndef SD_SELECT
        #define SD_SELECT BUILTIN_SDCARD
    #endif
    
    typedef std::map<uint8_t, std::string> ParamFilenameKey;
    namespace param_error
    {
        const uint8_t num_joint_ids = 3;
        const uint8_t SD_not_found_idx = num_joint_ids;
        const uint8_t file_not_found_idx = SD_not_found_idx + 1;
    }
    
    namespace controller_parameter_filenames
    {
        const ParamFilenameKey hip
        {
            // for disabled clear the parameters, may not want to use this if this is just a temp pause.  Same for zeroTorque
            {(uint8_t)config_defs::hip_controllers::disabled,"hipControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::zero_torque,"hipControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::heel_toe,"hipControllers/heelToe.csv"},
            {(uint8_t)config_defs::hip_controllers::extension_angle,"hipControllers/extensionAngle.csv"},
            {(uint8_t)config_defs::hip_controllers::franks_collins_hip, "hipControllers/franksCollins.csv"},
            {(uint8_t)config_defs::hip_controllers::bang_bang, "hipControllers/bangBang.csv"},
            {(uint8_t)config_defs::hip_controllers::user_defined, "hipControllers/userDefined.csv"},
            {(uint8_t)config_defs::hip_controllers::sine, "hipControllers/sine.csv"},
        };
        
        const ParamFilenameKey knee
        {
            {(uint8_t)config_defs::knee_controllers::disabled,"kneeControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::zero_torque,"kneeControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::user_defined, "kneeControllers/userDefined.csv"},
            {(uint8_t)config_defs::knee_controllers::sine, "kneeControllers/sine.csv"},
        };
        
        const ParamFilenameKey ankle
        {
            {(uint8_t)config_defs::ankle_controllers::disabled,"ankleControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::zero_torque,"ankleControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::pjmc,"ankleControllers/PJMC.csv"},
            {(uint8_t)config_defs::ankle_controllers::zhang_collins,"ankleControllers/zhangCollins.csv"},
            {(uint8_t)config_defs::ankle_controllers::user_defined, "ankleControllers/userDefined.csv"},
            {(uint8_t)config_defs::ankle_controllers::sine, "ankleControllers/sine.csv"},
        };
    };
    
    
    /*
     * Reads files from SD card and sets them to the appropriate controller parameters in the exo_data object
     * see ParseIni for details on inputs
     * 
     * joint_id : the joint id 
     * controller_id : the controller id 
     * set_num : parameter set to read from the SD card
     * exo_data : location to put the data 
     * 
     * Return : Error int.
     */
    uint8_t set_controller_params(uint8_t joint_id, uint8_t controller_id, uint8_t set_num, ExoData* exo_data);

#endif
#endif