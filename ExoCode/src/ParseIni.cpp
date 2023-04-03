#include "ParseIni.h"
#include "Logger.h"

// We only need to parse the INI file if we have access to the SD card.
// The nano will get the info through SPI so doesn't need these functions.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
    void ini_print_error_message(uint8_t e, bool eol = true)
    {
        if(Serial)
        {
            switch (e) {
            case IniFile::errorNoError:
                logger::print("no error");
                break;
            case IniFile::errorFileNotFound:
                logger::print("file not found");
                break;
            case IniFile::errorFileNotOpen:
                logger::print("file not open");
                break;
            case IniFile::errorBufferTooSmall:
                logger::print("buffer too small");
                break;
            case IniFile::errorSeekError:
                logger::print("seek error");
                break;
            case IniFile::errorSectionNotFound:
                logger::print("section not found");
                break;
            case IniFile::errorKeyNotFound:
                logger::print("key not found");
                break;
            case IniFile::errorEndOfFile:
                logger::print("end of file");
                break;
            case IniFile::errorUnknownError:
                logger::print("unknown error");
                break;
            default:
                logger::print("unknown error value");
                break;
            }
            if (eol)
            {
                logger::print("\n");
            }
        }
    }


    void ini_parser(uint8_t* config_to_send)
    {
         
        ini_parser("/config.ini", config_to_send);
        
    }


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
                logger::print("SD.begin() failed");
                logger::print("\n");
            }

        // Check the for the ini file
        IniFile ini(filename);
        if (!ini.open()) {
            if(Serial)
            {
                logger::print("Ini file ");
                logger::print(filename);
                logger::print(" does not exist");
                logger::print("\n");
            }
        // Cannot do anything else
            while (1);
        }
        if(Serial)
        {
            logger::print("Ini file exists");
            logger::print("\n");
        }
       

        // Check the file is valid. This can be used to warn if any lines
        // are longer than the buffer.
        if (!ini.validate(buffer, buffer_len)) {
            if(Serial)
            {
                logger::print("ini file ");
                logger::print(ini.getFilename());
                logger::print(" not valid: ");
                ini_print_error_message(ini.getError());
            }
            // Cannot do anything else
            while (1);
        }


          
        // I tried to make this iterable but gave up.
        // TODO:  Make this iterable 
        get_section_key(ini, "Board" , "name",  buffer, buffer_len); // read the key.
        data.board_name = buffer;  // store the value
        // logger::print(data.board_name.c_str());
        // logger::print("\t");
        // logger::println(config_map::board_name[data.board_name]);
        config_to_send[config_defs::board_name_idx] = config_map::board_name[data.board_name];  // encode the key to an uint8_t
        
        
        get_section_key(ini, "Board" , "version",  buffer, buffer_len);
        data.board_version = buffer;
        // logger::print(data.board_version.c_str());
        // logger::print("\t");
        // logger::println(config_map::board_version[data.board_version]);
        config_to_send[config_defs::board_version_idx] = config_map::board_version[data.board_version];
        
        //=========================================================
        
        get_section_key(ini, "Battery" , "name",  buffer, buffer_len);
        data.battery = buffer;
        // logger::print(data.board_version.c_str());
        // logger::print("\t");
        // logger::println(config_map::board_version[data.board_version]);
        config_to_send[config_defs::battery_idx] = config_map::battery[data.battery];
        
        //=========================================================
        
        get_section_key(ini, "Exo" , "name",  buffer, buffer_len);
        data.exo_name = buffer;
        // logger::print(data.exo_name.c_str());
        // logger::print("\t");
        // logger::println(config_map::exo_name[data.exo_name]);
        config_to_send[config_defs::exo_name_idx] = config_map::exo_name[data.exo_name];
        
        //=========================================================

        // Cast the string to a char array so get_section_key will can take it.
        const char temp_exo_name[data.exo_name.length()+1];
        strcpy(temp_exo_name,data.exo_name.c_str());

        // Check the section that corresponds to the exo_name to get the correct parameters.
        get_section_key(ini, temp_exo_name, "sides", buffer, buffer_len); 
        data.exo_sides = buffer;  
        // logger::print(data.exo_sides.c_str());
        // logger::print("\t");
        // logger::println(config_map::exo_side[data.exo_sides]);
        config_to_send[config_defs::exo_side_idx] = config_map::exo_side[data.exo_sides];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hip", buffer, buffer_len);
        data.exo_hip = buffer;
        // logger::print(data.exo_hip.c_str());
        // logger::print("\t");
        // logger::println(config_map::motor[data.exo_hip]);
        config_to_send[config_defs::hip_idx] = config_map::motor[data.exo_hip];
        
        get_section_key(ini, temp_exo_name, "knee", buffer, buffer_len);
        data.exo_knee = buffer;
        // logger::print(data.exo_knee.c_str());
        // logger::print("\t");
        // logger::println(config_map::motor[data.exo_knee]);
        config_to_send[config_defs::knee_idx] = config_map::motor[data.exo_knee];
        
        get_section_key(ini, temp_exo_name, "ankle", buffer, buffer_len);
        data.exo_ankle = buffer;
        // logger::print(data.exo_ankle.c_str());
        // logger::print("\t");
        // logger::println(config_map::motor[data.exo_ankle]);
        config_to_send[config_defs::ankle_idx] = config_map::motor[data.exo_ankle];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hipGearRatio", buffer, buffer_len);
        data.hip_gearing = buffer;
        // logger::print(data.hip_gearing.c_str());
        // logger::print("\t");
        // logger::println(config_map::motor[data.hip_gearing]);
        config_to_send[config_defs::hip_gear_idx] = config_map::gearing[data.hip_gearing];
        
        get_section_key(ini, temp_exo_name, "kneeGearRatio", buffer, buffer_len);
        data.knee_gearing = buffer;
        // logger::print(data.knee_gearing.c_str());
        // logger::print("\t");
        // logger::println(config_map::motor[data.knee_gearing]);
        config_to_send[config_defs::knee_gear_idx] = config_map::gearing[data.knee_gearing];
        
        get_section_key(ini, temp_exo_name, "ankleGearRatio", buffer, buffer_len);
        data.ankle_gearing = buffer;
        // logger::print(data.exo_ankle.c_str());
        // logger::print("\t");
        // logger::println(config_map::motor[data.exo_ankle]);
        config_to_send[config_defs::ankle_gear_idx] = config_map::gearing[data.ankle_gearing];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hipDefaultController", buffer, buffer_len);
        data.exo_hip_default_controller = buffer;
        // logger::print(data.exo_hip_default_controller.c_str());
        // logger::print("\t");
        // logger::println(config_map::hip_controllers[data.exo_hip_default_controller]);
        config_to_send[config_defs::exo_hip_default_controller_idx] = config_map::hip_controllers[data.exo_hip_default_controller];
        
        get_section_key(ini, temp_exo_name, "kneeDefaultController", buffer, buffer_len);
        data.exo_knee_default_controller = buffer;
        // logger::print(data.exo_knee_default_controller.c_str());
        // logger::print("\t");
        // logger::println(config_map::knee_controllers[data.exo_knee_default_controller]);
        config_to_send[config_defs::exo_knee_default_controller_idx] = config_map::knee_controllers[data.exo_knee_default_controller];
        
        get_section_key(ini, temp_exo_name, "ankleDefaultController", buffer, buffer_len);
        data.exo_ankle_default_controller = buffer;
        // logger::print(data.exo_ankle_default_controller.c_str());
        // logger::print("\t");
        // logger::println(config_map::ankle_controllers[data.exo_ankle_default_controller]);
        config_to_send[config_defs::exo_ankle_default_controller_idx] = config_map::ankle_controllers[data.exo_ankle_default_controller];
        
        //--------------------------------------------------------
        
        get_section_key(ini, temp_exo_name, "hipFlipDir", buffer, buffer_len);
        data.hip_flip_dir = buffer;
        // logger::print(data.hip_flip_dir.c_str());
        // logger::print("\t");
        // logger::println(config_map::flip_dir[data.hip_flip_dir]);
        config_to_send[config_defs::hip_flip_dir_idx] = config_map::flip_dir[data.hip_flip_dir];
        
        get_section_key(ini, temp_exo_name, "kneeFlipDir", buffer, buffer_len);
        data.knee_flip_dir = buffer;
        // logger::print(data.knee_flip_dir.c_str());
        // logger::print("\t");
        // logger::println(config_map::flip_dir[data.knee_flip_dir]);
        config_to_send[config_defs::knee_flip_dir_idx] = config_map::flip_dir[data.knee_flip_dir];
        
        get_section_key(ini, temp_exo_name, "ankleFlipDir", buffer, buffer_len);
        data.ankle_flip_dir = buffer;
        // logger::print(data.ankle_flip_dir.c_str());
        // logger::print("\t");
        // logger::println(config_map::flip_dir[data.ankle_flip_dir]);
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
                // logger::print("section '");
                // logger::print(section);
                // logger::print("' has an entry '");
                // logger::print(key);
                // logger::print("' with value ");
                // logger::print(buffer);
                // logger::print("\n");
            }
        }
        // Print the error if the key can't be found.
        else {
            if(Serial)
            {
                logger::print("Could not read '");
                logger::print(key);
                logger::print("' from section '");
                logger::print(section);
                logger::print("' , error was ");
                ini_print_error_message(ini.getError());
            }
        }
    }
#endif