/*
 * 
 * P. Stegall Jan. 2022
*/

#include "ParseIni.h"

// We only need to parse the INI file if we have access to the SD card.
// The nano will get the info through SPI so doesn't need these functions.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
    /*
     * ini_print_error_message(uint8_t e, bool eol = true)
     * 
     * Prints the error messages of IniFile object.
     * e is the error message, and eol true means a println will be used at the end.
     * 
     * Requires that Serial is defined.
     */
    void ini_print_error_message(uint8_t e, bool eol = true)
    {
        if(Serial)
        {
            switch (e) {
            case IniFile::errorNoError:
                Serial.print("no error");
                break;
            case IniFile::errorFileNotFound:
                Serial.print("file not found");
                break;
            case IniFile::errorFileNotOpen:
                Serial.print("file not open");
                break;
            case IniFile::errorBufferTooSmall:
                Serial.print("buffer too small");
                break;
            case IniFile::errorSeekError:
                Serial.print("seek error");
                break;
            case IniFile::errorSectionNotFound:
                Serial.print("section not found");
                break;
            case IniFile::errorKeyNotFound:
                Serial.print("key not found");
                break;
            case IniFile::errorEndOfFile:
                Serial.print("end of file");
                break;
            case IniFile::errorUnknownError:
                Serial.print("unknown error");
                break;
            default:
                Serial.print("unknown error value");
                break;
            }
            if (eol)
                Serial.print("\n");
        }
    }


    /*
     * configData ini_parser(*uint8_t config_to_send)
     * 
     * Parses the config.ini file in the root folder of the SD card and puts the parsed data to config_to_send
     * 
     */
    void ini_parser(uint8_t* config_to_send)
    {
         
        ini_parser("/config.ini", config_to_send);
        
    }


    /*
     * configData ini_parse(*char filename, *uint8_t config_to_send)
     * 
     * Parses the specified filename from the SD card and puts the parsed data to config_to_send
     * 
     */
    void ini_parser(char* filename, uint8_t* config_to_send)
    {
        ConfigData data;  // creates object to hold the key values

        // set pin to select the SD card
        pinMode(SD_SELECT, OUTPUT);
        digitalWrite(SD_SELECT, HIGH); // disable SD card

        // Create buffer to hold the data read from the file
        const size_t buffer_len = ini_config::buffer_length;
        char buffer[buffer_len];

        // setup the SPI to read the SD card
        SPI.begin();
        if (!SD.begin(SD_SELECT))
            while (1)
            if(Serial)
            {
                Serial.print("SD.begin() failed");
                Serial.print("\n");
            }

        // Check the for the ini file
        IniFile ini(filename);
        if (!ini.open()) {
            if(Serial)
            {
                Serial.print("Ini file ");
                Serial.print(filename);
                Serial.print(" does not exist");
                Serial.print("\n");
            }
        // Cannot do anything else
            while (1);
        }
        if(Serial)
        {
            //Serial.print("Ini file exists");
            //Serial.print("\n");
        }
       

        // Check the file is valid. This can be used to warn if any lines
        // are longer than the buffer.
        if (!ini.validate(buffer, buffer_len)) {
            if(Serial)
            {
                Serial.print("ini file ");
                Serial.print(ini.getFilename());
                Serial.print(" not valid: ");
                ini_print_error_message(ini.getError());
            }
            // Cannot do anything else
            while (1);
        }


          
        // I tried to make this iterable but gave up.
        // TODO:  Make this iterable 
        get_section_key(ini, "Board" , "name",  buffer, buffer_len); // read the key.
        data.board_name = buffer;  // store the value
        // Serial.print(data.board_name.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::board_name[data.board_name]);
        config_to_send[config_defs::board_name_idx] = config_map::board_name[data.board_name];  // encode the key to an uint8_t
        
        
        get_section_key(ini, "Board" , "version",  buffer, buffer_len);
        data.board_version = buffer;
        // Serial.print(data.board_version.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::board_version[data.board_version]);
        config_to_send[config_defs::board_version_idx] = config_map::board_version[data.board_version];
        
        //=========================================================
        
        get_section_key(ini, "Battery" , "name",  buffer, buffer_len);
        data.battery = buffer;
        // Serial.print(data.board_version.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::board_version[data.board_version]);
        config_to_send[config_defs::battery_idx] = config_map::battery[data.battery];
        
        //=========================================================
        
        get_section_key(ini, "Exo" , "name",  buffer, buffer_len);
        data.exo_name = buffer;
        // Serial.print(data.exo_name.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::exo_name[data.exo_name]);
        config_to_send[config_defs::exo_name_idx] = config_map::exo_name[data.exo_name];
        
        //=========================================================

        // Cast the string to a char array so get_section_key will can take it.
        const char temp_exo_name[data.exo_name.length()+1];
        strcpy(temp_exo_name,data.exo_name.c_str());

        // Check the section that corresponds to the exo_name to get the correct parameters.
        get_section_key(ini, temp_exo_name, "sides", buffer, buffer_len); 
        data.exo_sides = buffer;  
        // Serial.print(data.exo_sides.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::exo_side[data.exo_sides]);
        config_to_send[config_defs::exo_side_idx] = config_map::exo_side[data.exo_sides];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hip", buffer, buffer_len);
        data.exo_hip = buffer;
        // Serial.print(data.exo_hip.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::motor[data.exo_hip]);
        config_to_send[config_defs::hip_idx] = config_map::motor[data.exo_hip];
        
        get_section_key(ini, temp_exo_name, "knee", buffer, buffer_len);
        data.exo_knee = buffer;
        // Serial.print(data.exo_knee.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::motor[data.exo_knee]);
        config_to_send[config_defs::knee_idx] = config_map::motor[data.exo_knee];
        
        get_section_key(ini, temp_exo_name, "ankle", buffer, buffer_len);
        data.exo_ankle = buffer;
        // Serial.print(data.exo_ankle.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::motor[data.exo_ankle]);
        config_to_send[config_defs::ankle_idx] = config_map::motor[data.exo_ankle];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hip", buffer, buffer_len);
        data.exo_hip = buffer;
        // Serial.print(data.exo_hip.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::motor[data.exo_hip]);
        config_to_send[config_defs::hip_idx] = config_map::motor[data.exo_hip];
        
        get_section_key(ini, temp_exo_name, "knee", buffer, buffer_len);
        data.exo_knee = buffer;
        // Serial.print(data.exo_knee.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::motor[data.exo_knee]);
        config_to_send[config_defs::knee_idx] = config_map::motor[data.exo_knee];
        
        get_section_key(ini, temp_exo_name, "ankle", buffer, buffer_len);
        data.exo_ankle = buffer;
        // Serial.print(data.exo_ankle.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::motor[data.exo_ankle]);
        config_to_send[config_defs::ankle_idx] = config_map::motor[data.exo_ankle];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hipDefaultController", buffer, buffer_len);
        data.exo_hip_default_controller = buffer;
        // Serial.print(data.exo_hip_default_controller.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::hip_controllers[data.exo_hip_default_controller]);
        config_to_send[config_defs::exo_hip_default_controller_idx] = config_map::hip_controllers[data.exo_hip_default_controller];
        
        get_section_key(ini, temp_exo_name, "kneeDefaultController", buffer, buffer_len);
        data.exo_knee_default_controller = buffer;
        // Serial.print(data.exo_knee_default_controller.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::knee_controllers[data.exo_knee_default_controller]);
        config_to_send[config_defs::exo_knee_default_controller_idx] = config_map::knee_controllers[data.exo_knee_default_controller];
        
        get_section_key(ini, temp_exo_name, "ankleDefaultController", buffer, buffer_len);
        data.exo_ankle_default_controller = buffer;
        // Serial.print(data.exo_ankle_default_controller.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::ankle_controllers[data.exo_ankle_default_controller]);
        config_to_send[config_defs::exo_ankle_default_controller_idx] = config_map::ankle_controllers[data.exo_ankle_default_controller];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hipFlipDir", buffer, buffer_len);
        data.hip_flip_dir = buffer;
        // Serial.print(data.hip_flip_dir.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::flip_dir[data.hip_flip_dir]);
        config_to_send[config_defs::hip_flip_dir_idx] = config_map::flip_dir[data.hip_flip_dir];
        
        get_section_key(ini, temp_exo_name, "kneeFlipDir", buffer, buffer_len);
        data.knee_flip_dir = buffer;
        // Serial.print(data.knee_flip_dir.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::flip_dir[data.knee_flip_dir]);
        config_to_send[config_defs::knee_flip_dir_idx] = config_map::flip_dir[data.knee_flip_dir];
        
        get_section_key(ini, temp_exo_name, "ankleFlipDir", buffer, buffer_len);
        data.ankle_flip_dir = buffer;
        // Serial.print(data.ankle_flip_dir.c_str());
        // Serial.print("\t");
        // Serial.println(config_map::flip_dir[data.ankle_flip_dir]);
        config_to_send[config_defs::ankle_flip_dir_idx] = config_map::flip_dir[data.ankle_flip_dir];
    }

    /*
     * void get_section_key(IniFile ini, const char* section, const char* key, char* buffer, size_t buffer_len)
     * 
     * retrieve the key from the ini file, and put it in the buffer.
     * Also prints the value or the error if it can't find the key.
     * 
     * Requires Serial exists.
     */
    void get_section_key(IniFile ini, const char* section, const char* key, char* buffer, size_t buffer_len)
    {

        // Fetch a value from a key which is present.
        if (ini.getValue(section, key, buffer, buffer_len)) {
            if(Serial)
            {
                // Serial.print("section '");
                // Serial.print(section);
                // Serial.print("' has an entry '");
                // Serial.print(key);
                // Serial.print("' with value ");
                // Serial.print(buffer);
                // Serial.print("\n");
            }
        }
        // Print the error if the key can't be found.
        else {
            if(Serial)
            {
                Serial.print("Could not read '");
                Serial.print(key);
                Serial.print("' from section '");
                Serial.print(section);
                Serial.print("' , error was ");
                ini_print_error_message(ini.getError());
            }
        }
    }
#endif