/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef ParseIni_h
#define ParseIni_h

// used for uint8_t
#include <stdint.h>

// define the constants to use for the various arrays.
namespace ini_config{
    const int buffer_length = 500;
    const int key_length = 25;
    const int section_length = 10;
    const int number_of_keys = 10;
    //const char *config_filename = "/config.ini";  // this line creates an "Error compiling for board Teensy 3.6." so I am just hard coding it.
};

// Includes for reading the ini file from the SD card.

namespace config_defs
{
    enum class board_name : uint8_t
    {
        AK_board = 1,
    };
    
    enum class board_version : uint8_t
    { 
        zero_one = 1,
    };

    enum class exo_name : uint8_t
    { 
        bilateral_ankle = 1, 
        bilateral_hip = 2, 
        bilateral_hip_ankle = 3,
        left_ankle = 4,
        right_ankle = 5,
        left_hip = 6,
        right_hip = 7,
    };
    
    enum class exo_side : uint8_t
    { 
        bilateral = 1, 
        left = 2, 
        right = 3,
    };
    
    enum class JointType
    {
        hip = 0,
        knee = 1,
        ankle = 2,
    };
    
    enum class motor : uint8_t
    { 
        not_used = 1, 
        AK60 = 2, 
        AK80 = 3,
    };
    
    enum class joint_id : uint8_t
    {
        // byte format : [is_left, !is_left, unused_joint, unused_joint, unused_joint, is_ankle, is_knee, is_hip]
        left = 0b10000000,
        right = 0b01000000,
        
        hip = 0b00000001,
        knee = 0b00000010,
        ankle = 0b00000100,
                
        left_hip = left|hip,
        left_knee = left|knee,
        left_ankle = left|ankle,
        
        right_hip = right|hip,
        right_knee = right|knee,
        right_ankle = right|ankle,
        
    };
        
    enum class  hip_controllers : uint8_t
    { 
        disabled = 1, 
        zero_torque = 2, 
        heel_toe =  3,
        extension_angle = 4,
    };
    
    enum class knee_controllers : uint8_t
    { 
        disabled = 1, 
        zero_torque = 2,
    };
        
    enum class ankle_controllers : uint8_t
    {
        disabled = 1, 
        zero_torque = 2, 
        pjmc = 3,
        zhang_collins = 4,
    };
    
    static const int board_name_idx = 0;
    static const int board_version_idx = 1;
    
    static const int exo_name_idx = 2;
    static const int exo_side_idx = 3;
    
    static const int hip_idx = 4;
    static const int knee_idx = 5;
    static const int ankle_idx = 6;
    
    static const int exo_hip_default_controller_idx = 7;
    static const int exo_knee_default_controller_idx = 8;
    static const int exo_ankle_default_controller_idx = 9;
}

#if defined(ARDUINO_TEENSY36) 
    #include <SD.h>
    #include <SPI.h>
    #include "IniFile.h"

    

    // Need to install ArduinoSTL in the library manager to use map
    #include <map>
    #include <string>



    // The select pin used for the SD card
    #define SD_SELECT BUILTIN_SDCARD

    




    void ini_parser(uint8_t* config_to_send); // uses default filename
    void ini_parser(char* filename, uint8_t* config_to_send); //uses sent filename
    // TODO: create non-verbose version of get section keys.
    void get_section_key(IniFile ini, const char* section, const char* key, char* buffer, size_t buffer_len);  //retrieve the key values and get the print the output
    void ini_print_error_message(uint8_t e, bool eol = true); 


    /*
    Mappings of config names to uint8_t that will be sent to the nano
    "0" is mapped to one to help with debug as we should never send all zeros in the bytes.
    If you see a uint8_t that is zero it indicates the field didn't exist.
    */
    namespace config_map
    {  
        


        // define our own type so we don't have to type so much
        typedef std::map<std::string, uint8_t> IniKeyCode;
        
        const IniKeyCode board_name = {{"AK_Board", (uint8_t)config_defs::board_name::AK_board},};
        const IniKeyCode board_version = { {"0.1", (uint8_t)config_defs::board_version::zero_one}, };

        const IniKeyCode exo_name 
        { 
            {"bilateralAnkle", (uint8_t)config_defs::exo_name::bilateral_ankle}, 
            {"bilateralHip", (uint8_t)config_defs::exo_name::bilateral_hip}, 
            {"bilateralHipAnkle", (uint8_t)config_defs::exo_name::bilateral_hip_ankle},
            {"leftAnkle", (uint8_t)config_defs::exo_name::left_ankle},
            {"rightAnkle", (uint8_t)config_defs::exo_name::right_ankle},
            {"leftHip", (uint8_t)config_defs::exo_name::left_hip},
            {"rightHip", (uint8_t)config_defs::exo_name::right_hip},
        };
        
        const IniKeyCode exo_side 
        { 
            {"bilateral", (uint8_t)config_defs::exo_side::bilateral}, 
            {"left", (uint8_t)config_defs::exo_side::left}, 
            {"right", (uint8_t)config_defs::exo_side::right},
        };
        
        const IniKeyCode motor 
        { 
            {"0", (uint8_t)config_defs::motor::not_used}, 
            {"AK60", (uint8_t)config_defs::motor::AK60}, 
            {"AK80", (uint8_t)config_defs::motor::AK80},
        };
        
        
        const IniKeyCode hip_controllers 
        { 
            {"0", (uint8_t)config_defs::hip_controllers::disabled}, 
            {"zeroTorque", (uint8_t)config_defs::hip_controllers::zero_torque}, 
            {"4*toe-heel", (uint8_t)config_defs::hip_controllers::heel_toe},
            {"extensionAngle", (uint8_t)config_defs::hip_controllers::extension_angle},
        };
        
        const IniKeyCode knee_controllers 
        { 
            {"0", (uint8_t)config_defs::knee_controllers::disabled}, 
            {"zeroTorque", (uint8_t)config_defs::knee_controllers::zero_torque}, 
        };
        
        const IniKeyCode ankle_controllers 
        { 
            {"0", (uint8_t)config_defs::ankle_controllers::disabled}, 
            {"zeroTorque", (uint8_t)config_defs::ankle_controllers::zero_torque}, 
            {"PJMC", (uint8_t)config_defs::ankle_controllers::pjmc},
            {"zhangCollins", (uint8_t)config_defs::ankle_controllers::zhang_collins},
            
        };  
    };




    /*
     * Holds the raw key value strings from the ini file
     */
    struct ConfigData{
        
        std::string board_name;
        std::string board_version;
        
        std::string exo_name;
        std::string exo_sides;
        
        std::string exo_hip;
        std::string exo_knee;
        std::string exo_ankle;
        
        std::string exo_hip_default_controller;
        std::string exo_knee_default_controller;
        std::string exo_ankle_default_controller;
        
        
    };
#endif


#endif
