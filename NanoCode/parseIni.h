/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef ParseIni_h
#define ParseIni_h

// Includes for reading the ini file from the SD card.
#include <SD.h>
#include <SPI.h>
#include "IniFile.h"

// Need to install ArduinoSTL in the library manager to use map
#include <map>
#include <string>

// used for uint8_t
#include <stdint.h>

// The select pin used for the SD card
#define SD_SELECT BUILTIN_SDCARD

// define the constants to use for the various arrays.
namespace ini_config{
    const int buffer_length = 500;
    const int key_length = 25;
    const int section_length = 10;
    const int number_of_keys = 10;
    //const char *config_filename = "/config.ini";  // this line creates an "Error compiling for board Teensy 3.6." so I am just hard coding it.
};




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
    
    const IniKeyCode board_name = {{"AK_Board", 1},};
    const IniKeyCode board_version = { {"0.1", 1}, };

    const IniKeyCode exo_name 
    { 
        {"bilateralAnkle", 1}, 
        {"bilateralHip", 2}, 
        {"bilateralHipAnkle", 3},
        {"leftAnkle", 4},
        {"rightAnkle", 5},
        {"leftHip", 6},
        {"rightHip", 7},
    };
    
    const IniKeyCode exo_side 
    { 
        {"bilateral", 1}, 
        {"left", 2}, 
        {"right", 3},
    };
    
    const IniKeyCode motor 
    { 
        {"0", 1}, 
        {"AK60", 2}, 
        {"AK80", 3},
    };
    
    
    const IniKeyCode hip_controllers 
    { 
        {"0", 1}, 
        {"zeroTorque", 2}, 
        {"4*toe-heel", 3},
    };
    
    const IniKeyCode knee_controllers 
    { 
        {"0", 1}, 
        {"zeroTorque", 2}, 
    };
    
    const IniKeyCode ankle_controllers 
    { 
        {"0", 1}, 
        {"zeroTorque", 2}, 
        {"PJMC", 3},
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
