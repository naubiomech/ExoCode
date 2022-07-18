/*
 * 
 * 
 * P. Stegall July 2022
*/

#include "ParamsFromSD.h"
// #define SD_PARAM_DEBUG 1

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

    // // returns 0 if cell is empty or eol
    // bool read_cell(File* file, char delim, char* buff, uint8_t buff_len)
    // {
        // char ch;
    // }
    
    // unsigned long go_to_eol(File file)
    // {
        // char ch;
        // while (ch != '\n' || ch == -1)
        // {
            // ch = file.read()
        // }
        // return file.position;
    // }
    
    uint8_t set_controller_params(uint8_t joint_id, uint8_t controller_id, uint8_t set_num, ExoData* exo_data)
    {   
        // SD inherits from stream which has a lot more useful methods that we will use.
        File param_file;
        std::string filename;
        uint8_t header_size;
        uint8_t param_num_in_file;
        uint8_t line_to_read;
        uint8_t error_type;
        //uint8_t cell_size = 256; //max num chars in a cell
        // char value[cell_size];
        // char end_of_line = '\n';
        // char delim = ',';
        // uint8_t char_num = 0;
        
        switch(utils::get_joint_type(joint_id))
        {
            case (uint8_t)config_defs::joint_id::hip:
            {
                
                #ifdef SD_PARAM_DEBUG
                    Serial.println("\n\nset_controller_params : Hip");
                #endif
                // Connect to SD card
                SPI.begin();
                #ifdef SD_PARAM_DEBUG
                    Serial.println("set_controller_params : SPI Begin");
                #endif
                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::hip, 1, param_error::SD_not_found_idx);
                    #ifdef SD_PARAM_DEBUG
                        Serial.println("set_controller_params : SD Not Found");
                    #endif
                    return error_type;
                }
                else 
                {
                    // Get filename
                    filename = controller_parameter_filenames::hip[controller_id];
                    #ifdef SD_PARAM_DEBUG
                        Serial.print("set_controller_params : filename = ");
                        Serial.println(filename.c_str());
                    #endif
                    // Open File
                    param_file = SD.open(filename.c_str(), FILE_READ);
                    #ifdef SD_PARAM_DEBUG
                        Serial.print("set_controller_params : ");
                        Serial.print(filename.c_str());
                        Serial.println(" opened");
                    #endif
                    // check file exists
                    if (param_file)
                    {   
                        while(param_file.available())
                        {
                            // First value should be header size
                            header_size = param_file.parseInt();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : header size ");
                                Serial.println(header_size);
                            #endif
                            // skip to the line we need
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            { 
                                // first value in second line should be parameter number
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();
                                    #ifdef SD_PARAM_DEBUG
                                        Serial.print("set_controller_params : Number of parameters in file = ");
                                        Serial.println(param_num_in_file);
                                    #endif    
                                } 
                                // keep going through the file till the next new line.  This is so it will restart if timeout happens.          
                                while(!param_file.findUntil('\n','\n'))
                                {
                                    ;
                                }
                                #ifdef SD_PARAM_DEBUG
                                    Serial.print("set_controller_params : read line ");
                                    Serial.println(line_being_read);
                                #endif
                            }
                            
                            // store the line start value so we can go back here
                            unsigned long line_start = param_file.position();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : parameter set start ");
                                Serial.println(line_start);
                            #endif
                            
                            // find the end of the line
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : parameter set end ");
                                Serial.println(line_end);
                            #endif
                            
                            // reset to the start of the line
                            param_file.seek(line_start);
                            #ifdef SD_PARAM_DEBUG
                                Serial.println("set_controller_params : reset to line start");
                            #endif
                            // set the parameters.
                            uint8_t param_num = 0;
                            float read_val = 0;
                            if(utils::get_is_left(joint_id))
                            {
                                #ifdef SD_PARAM_DEBUG
                                    Serial.println("set_controller_params : is Left ");
                                #endif
                                
                                // read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("+Value in file :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("-File Line Ended :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                    }
                                    
                                    exo_data->left_leg.hip.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            else
                            {
                                #ifdef SD_PARAM_DEBUG
                                    Serial.println("set_controller_params : is Right ");
                                #endif
                                
                                // read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("+Value in file :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("-File Line Ended :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                    }
                                    
                                    exo_data->right_leg.hip.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            // we don't need to read the rest of the file
                            break;
                        }
                        
                    }
                    else
                    { 
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::hip, 1, param_error::file_not_found_idx);
                        #ifdef SD_PARAM_DEBUG
                            Serial.println("set_controller_params : File not found");
                        #endif
                    }
                    param_file.close();
                    #ifdef SD_PARAM_DEBUG
                        Serial.println("set_controller_params : File Closed");
                    #endif
                }
                break;
            }
            case (uint8_t)config_defs::joint_id::knee:
            {
                #ifdef SD_PARAM_DEBUG
                    Serial.println("\n\nset_controller_params : Knee");
                #endif
                // Connect to SD card
                SPI.begin();
                #ifdef SD_PARAM_DEBUG
                    Serial.println("set_controller_params : SPI Begin");
                #endif
                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::knee, 1, param_error::SD_not_found_idx);
                    #ifdef SD_PARAM_DEBUG
                        Serial.println("set_controller_params : SD Not Found");
                    #endif
                    return error_type;
                }
                else 
                {
                    // Get filename
                    filename = controller_parameter_filenames::knee[controller_id];
                    #ifdef SD_PARAM_DEBUG
                        Serial.print("set_controller_params : filename = ");
                        Serial.println(filename.c_str());
                    #endif
                    // Open File
                    param_file = SD.open(filename.c_str(), FILE_READ);
                    #ifdef SD_PARAM_DEBUG
                        Serial.print("set_controller_params : ");
                        Serial.print(filename.c_str());
                        Serial.println(" opened");
                    #endif
                    // check file exists
                    if (param_file)
                    {   
                        while(param_file.available())
                        {
                            // First value should be header size
                            header_size = param_file.parseInt();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : header size ");
                                Serial.println(header_size);
                            #endif
                            // skip to the line we need
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            { 
                                // first value in second line should be parameter number
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();
                                    #ifdef SD_PARAM_DEBUG
                                        Serial.print("set_controller_params : Number of parameters in file = ");
                                        Serial.println(param_num_in_file);
                                    #endif    
                                } 
                                // keep going through the file till the next new line.  This is so it will restart if timeout happens.          
                                while(!param_file.findUntil('\n','\n'))
                                {
                                    ;
                                }
                                #ifdef SD_PARAM_DEBUG
                                    Serial.print("set_controller_params : read line ");
                                    Serial.println(line_being_read);
                                #endif
                            }
                            
                            // store the line start value so we can go back here
                            unsigned long line_start = param_file.position();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : parameter set start ");
                                Serial.println(line_start);
                            #endif
                            
                            // find the end of the line
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : parameter set end ");
                                Serial.println(line_end);
                            #endif
                            
                            // reset to the start of the line
                            param_file.seek(line_start);
                            #ifdef SD_PARAM_DEBUG
                                Serial.println("set_controller_params : reset to line start");
                            #endif
                            // set the parameters.
                            uint8_t param_num = 0;
                            float read_val = 0;
                            if(utils::get_is_left(joint_id))
                            {
                                #ifdef SD_PARAM_DEBUG
                                    Serial.println("set_controller_params : is Left ");
                                #endif
                                
                                // read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("+Value in file :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("-File Line Ended :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                    }
                                    
                                    exo_data->left_leg.knee.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            else
                            {
                                #ifdef SD_PARAM_DEBUG
                                    Serial.println("set_controller_params : is Right ");
                                #endif
                                
                                // read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("+Value in file :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("-File Line Ended :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                    }
                                    
                                    exo_data->right_leg.knee.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            // we don't need to read the rest of the file
                            break;
                        }
                        
                    }
                    else
                    { 
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::knee, 1, param_error::file_not_found_idx);
                        #ifdef SD_PARAM_DEBUG
                            Serial.println("set_controller_params : File not found");
                        #endif
                    }
                    param_file.close();
                    #ifdef SD_PARAM_DEBUG
                        Serial.println("set_controller_params : File Closed");
                    #endif
                }
                break;
            }
            case (uint8_t)config_defs::joint_id::ankle:
            {
                #ifdef SD_PARAM_DEBUG
                    Serial.println("\n\nset_controller_params : Ankle");
                #endif
                // Connect to SD card
                SPI.begin();
                #ifdef SD_PARAM_DEBUG
                    Serial.println("set_controller_params : SPI Begin");
                #endif
                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::ankle, 1, param_error::SD_not_found_idx);
                    #ifdef SD_PARAM_DEBUG
                        Serial.println("set_controller_params : SD Not Found");
                    #endif
                    return error_type;
                }
                else 
                {
                    // Get filename
                    filename = controller_parameter_filenames::ankle[controller_id];
                    #ifdef SD_PARAM_DEBUG
                        Serial.print("set_controller_params : filename = ");
                        Serial.println(filename.c_str());
                    #endif
                    // Open File
                    param_file = SD.open(filename.c_str(), FILE_READ);
                    #ifdef SD_PARAM_DEBUG
                        Serial.print("set_controller_params : ");
                        Serial.print(filename.c_str());
                        Serial.println(" opened");
                    #endif
                    // check file exists
                    if (param_file)
                    {   
                        while(param_file.available())
                        {
                            // First value should be header size
                            header_size = param_file.parseInt();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : header size ");
                                Serial.println(header_size);
                            #endif
                            // skip to the line we need
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            { 
                                // first value in second line should be parameter number
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();
                                    #ifdef SD_PARAM_DEBUG
                                        Serial.print("set_controller_params : Number of parameters in file = ");
                                        Serial.println(param_num_in_file);
                                    #endif    
                                } 
                                // keep going through the file till the next new line.  This is so it will restart if timeout happens.          
                                while(!param_file.findUntil('\n','\n'))
                                {
                                    ;
                                }
                                #ifdef SD_PARAM_DEBUG
                                    Serial.print("set_controller_params : read line ");
                                    Serial.println(line_being_read);
                                #endif
                            }
                            
                            // store the line start value so we can go back here
                            unsigned long line_start = param_file.position();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : parameter set start ");
                                Serial.println(line_start);
                            #endif
                            
                            // find the end of the line
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();
                            #ifdef SD_PARAM_DEBUG
                                Serial.print("set_controller_params : parameter set end ");
                                Serial.println(line_end);
                            #endif
                            
                            // reset to the start of the line
                            param_file.seek(line_start);
                            #ifdef SD_PARAM_DEBUG
                                Serial.println("set_controller_params : reset to line start");
                            #endif
                            // set the parameters.
                            uint8_t param_num = 0;
                            float read_val = 0;
                            if(utils::get_is_left(joint_id))
                            {
                                #ifdef SD_PARAM_DEBUG
                                    Serial.println("set_controller_params : is Left ");
                                #endif
                                
                                // read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("+Value in file :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("-File Line Ended :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                    }
                                    
                                    exo_data->left_leg.ankle.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            else
                            {
                                #ifdef SD_PARAM_DEBUG
                                    Serial.println("set_controller_params : is Right ");
                                #endif
                                
                                // read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("+Value in file :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;
                                        #ifdef SD_PARAM_DEBUG
                                            Serial.print("-File Line Ended :\t");
                                            Serial.println(read_val,6);
                                        #endif
                                    }
                                    
                                    exo_data->right_leg.ankle.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            // we don't need to read the rest of the file
                            break;
                        }
                        
                    }
                    else
                    { 
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::ankle, 1, param_error::file_not_found_idx);
                        #ifdef SD_PARAM_DEBUG
                            Serial.println("set_controller_params : File not found");
                        #endif
                    }
                    param_file.close();
                    #ifdef SD_PARAM_DEBUG
                        Serial.println("set_controller_params : File Closed");
                    #endif
                }
                break;
            }
            
        }
        return error_type;
    }

#endif