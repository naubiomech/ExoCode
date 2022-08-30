/*
 * 
 * P. Stegall Jan. 2022
*/

#include "SPIHandler.h"
#include "Arduino.h"
#include <vector>

#define SPI_DEBUG
//#define SPI_DEBUG_RAW 
// TODO::update status to uint16_t


/*
 * Takes in the in a byte and if it is 0xFF changes to 0xFE
 * Takes in a byte array that records if the FE's are really FF(1) or FE(0)
 * Takes in the current bit for is_ff and increments if we have written is_ff
 * Takes in the current byte for is_ff and increments if we have written is_ff
 *      if current bit reaches 7 rolls over to 0 since we can only go as high as F0xFE
 */
void ff_to_fe(uint8_t* val, uint8_t* is_ff, uint8_t* current_byte, uint8_t* current_bit)
{
    if(0xFF==*val)
    {
        *val = 0xFE;
        is_ff[*current_byte] = utils::update_bit(is_ff[*current_byte], 1, *current_bit);
        *current_bit++;
    }
    else if(0xFE==*val)
    {
        is_ff[*current_byte] = utils::update_bit(is_ff[*current_byte], 0, *current_bit);
        *current_bit++;
    }
    
    if (*current_bit>=7)
    {
        *current_bit=0;
        *current_byte++;
    }
    
    return;
}

/*
 * Takes in the in a byte and if it is 0xFE and is supposed to be 0xFF changes to 0xFF
 * Takes in a byte that records if the FE's are really FF(1) or FE(0)
 * Takes in the current bit for is_ff and increments if the value was 0xFE
 */
void fe_to_ff(uint8_t* val, uint8_t* is_ff, uint8_t* current_byte, uint8_t* current_bit)
{
    if(0xFE==*val)
    {
        if (utils::get_bit(is_ff[*current_byte], *current_bit)) 
        {
            *val = 0xFF;
        }
        *current_bit++;
        if (*current_bit>=7)
        {
            *current_bit=0;
            *current_byte++;
        }
    }
    return;
}
uint8_t ff_to_fe_check(uint8_t val)
{
    //updating val doesn't change it in the source location
    //utils::ff_to_fe(&val,spi_data_idx::is_ff::is_ff,&(spi_data_idx::is_ff::current_byte),&(spi_data_idx::is_ff::current_bit));
    
    if(0xFF==val)
    {
        val = 0xFE;
        spi_data_idx::is_ff::is_ff[spi_data_idx::is_ff::current_byte] = utils::update_bit(spi_data_idx::is_ff::is_ff[spi_data_idx::is_ff::current_byte], 1, spi_data_idx::is_ff::current_bit);
        spi_data_idx::is_ff::current_bit++;
    }
    else if(0xFE==val)
    {
        spi_data_idx::is_ff::is_ff[spi_data_idx::is_ff::current_byte] = utils::update_bit(spi_data_idx::is_ff::is_ff[spi_data_idx::is_ff::current_byte], 0, spi_data_idx::is_ff::current_bit);
        spi_data_idx::is_ff::current_bit++;
    }
    
    if (spi_data_idx::is_ff::current_bit>=7)
    {
        spi_data_idx::is_ff::current_bit=0;
        spi_data_idx::is_ff::current_byte++;
    }
    return val;
};

uint8_t fe_to_ff_check(uint8_t val)
{
    //updating val doesn't change it in the source location
    //utils::fe_to_ff(&val,spi_data_idx::is_ff::is_ff,&(spi_data_idx::is_ff::current_byte),&(spi_data_idx::is_ff::current_bit));
    #ifdef SPI_DEBUG
        //Serial.println("fe_to_ff_check : ");
    #endif
    if(0xFE==val)
    {
        if (utils::get_bit(spi_data_idx::is_ff::is_ff[spi_data_idx::is_ff::current_byte], spi_data_idx::is_ff::current_bit)) 
        {
            val = 0xFF;
            #ifdef SPI_DEBUG
                // Serial.println("\tVal is FF");
            #endif
        }
        else
        {
            #ifdef SPI_DEBUG
                // Serial.println("\tVal is FE");
            #endif
        }
        spi_data_idx::is_ff::current_bit++;
        if (spi_data_idx::is_ff::current_bit>=7)
        {
            spi_data_idx::is_ff::current_bit=0;
            spi_data_idx::is_ff::current_byte++;
        }
    }
    
    return val;
};

void print_message_name(uint8_t msg_id)
{
    switch (msg_id)
    {
        case spi_cmd::null_cmd::id :
            Serial.print("Null");
            break;
        case spi_cmd::send_config::id :
            Serial.print("Send Config");
            break;
        case spi_cmd::send_data_exo::id :
            Serial.print("Send Data Exo");
            break;
        case spi_cmd::update_controller::id :
            Serial.print("Update Controller");
            break;
        case spi_cmd::update_controller_params::id :
            Serial.print("Update Controller Parameters");
            break;
        case spi_cmd::calibrate_torque_sensor::id :
            Serial.print("Calibrate Torque Sensor");
            break;
        case spi_cmd::calibrate_fsr::id :
            Serial.print("Calibrate FSR");
            break;
        case spi_cmd::refine_fsr::id :
            Serial.print("Refine FSR");
            break;
        case spi_cmd::motor_enable_disable::id :
            Serial.print("Motor Enable Disable");
            break;
        case spi_cmd::motor_zero::id :
            Serial.print("Motor Zero");
            break;
    }
};
    
void print_config(uint8_t* config_to_send)
{
    Serial.println("print_config()");
    for(int i = 0; i<ini_config::number_of_keys; i++)
    {
        Serial.print("\t[");
        Serial.print(i);
        Serial.print("] : ");
        Serial.println(config_to_send[i]);
    }
};

void print_message(uint8_t* message, uint8_t len)
{
    for( int i = 0; i<len; i++)
    {
        Serial.print("\t[");
        Serial.print(i);
        Serial.print("] -> 0x");
        Serial.println(message[i],HEX);
    }
    Serial.println("");
};
       
uint8_t get_data_len(uint8_t* config_to_send)
{
    uint8_t num_legs = (config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::bilateral) ? 2 : (((config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::left) || (config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::right)) ? 1 : 0);
    #ifdef SPI_DEBUG
        // Serial.print("get_data_len : num_legs = ");
        // Serial.println(num_legs);
    #endif
    uint8_t num_joints_per_leg = ((config_to_send[config_defs::hip_idx] == (uint8_t)config_defs::motor::not_used) ? 0 : 1) + ((config_to_send[config_defs::knee_idx] == (uint8_t)config_defs::motor::not_used) ? 0 : 1) + ((config_to_send[config_defs::ankle_idx] == (uint8_t)config_defs::motor::not_used) ? 0 : 1); 
    #ifdef SPI_DEBUG
        // Serial.print("get_data_len : num_joints_per_leg = ");
        // Serial.println(num_joints_per_leg);
    #endif
    // number of parameters as floats times the number of bytes in a float, or the number of byte in the config.  Whichever is bigger.
    return  (spi_data_idx::exo::idx_cnt + num_legs * (spi_data_idx::leg::idx_cnt + num_joints_per_leg * (spi_data_idx::joint::idx_cnt + spi_data_idx::motor::idx_cnt)));

};
namespace spi_data_idx // read data that changes each loop
{
    namespace is_ff
    {
        uint8_t is_ff[num_bytes] = {0};
        uint8_t current_byte = 0;
        uint8_t current_bit = 0;
        bool overflow = false;
        
        void clear_is_ff()
        {
            current_byte = 0;
            current_bit = 0;
            overflow = false;
            for(uint8_t i = 0;i < num_bytes; i++)
            {
                is_ff[i] = 0;
            }
        };
        
        bool overflowcheck()
        {
            if(current_byte >= num_bytes && current_bit > 0)
            {
                overflow = true;
            }
            return overflow;
        };
        
    };
};


#if defined(ARDUINO_TEENSY36)
    #include "TSPISlave.h"
    
    void spi_msg(TSPISlave my_spi, uint16_t *buffer)
    {
        //This is just a message to test.  It should be replaced with the message based on what the msg was.
        static uint16_t cnt = 0;
        cnt++;
        buffer[0] = 0x1111 + cnt;
        buffer[1] = 0x2222 + cnt;
        buffer[2] = 0x3333 + cnt;
        
        uint8_t i = 0;  // track the index of the loop
        uint8_t msg = 0;  // store the message id that comes in
        uint8_t len = 0;  // the length of the message, not sure how much this will be needed.
        uint16_t full = 0;  // stores the msg/len combo in the first uint16_t sent
        uint16_t joint_id = 0;  // stores the joint id you want the data from, I was in the process of adding this.
        Serial.println("");
        Serial.println("");
        Serial.println(millis());
        
        while (my_spi.active() ) 
        {
            if (my_spi.available()) 
            {
                if (i==0)
                {
                    // read the first uint16_t and parse it
                    full = my_spi.popr();  
                    msg = full>>8;
                    len = full & 0xFF;
                    //joint_id = my_spi.popr(); // pull in the joint id
                    
                    Serial.print("message id : ");
                    Serial.println(msg);
                    Serial.print("message length : ");
                    Serial.println(len);
                    //Serial.print("joint id : ");
                    //Serial.println(joint_id);
                    //Serial.print("Sent CNT: 0x");
                    //Serial.println(++cnt, HEX);
                   // my_spi.pushr(cnt);
                   
                   // these are needed to shift things correctly for the response.  We can probably get rid of it when we use are doing the call response as the earlier message response can start at the beginning.
                    my_spi.pushr(0xBB);
                    my_spi.pushr(0xCC);
                    my_spi.pushr(0xDD);
                    
                }
                
                else
                {
                    // Send the message starting when i=1
                    Serial.println("");
                    Serial.print("Sent VALUE: 0x");
                    Serial.println(buffer[i-1], HEX);
                    my_spi.pushr(buffer[i-1]);
                    
                    // swap the received message into the buffer.
                    Serial.print("Recv VALUE: 0x");
                    buffer[i-1] = my_spi.popr();
                    Serial.println(buffer[i-1], HEX);
                }
                i++;
            }
        }
        //Serial.println("END"); 
    }

#elif defined(ARDUINO_TEENSY41)
    #include "SPISlave_T4.h"
    
    // ===========================================================================================================  
    namespace static_spi_handler
    {   
        uint8_t peripheral_transaction(SPISlave_T4<&SPI, SPI_8_BITS> my_spi, uint8_t* config_to_send, ExoData* data)
        {
            uint8_t msg_len = padding + max(max(spi_cmd::max_param_len, get_data_len(config_to_send)), ini_config::number_of_keys) + spi_data_idx::is_ff::num_bytes;
        
            uint8_t controller_message[msg_len];
            uint8_t peripheral_message[msg_len];
            
            #ifdef SPI_DEBUG_RAW
                // Serial.println("\n\nstatic_spi_handler::peripheral_transaction : \n#######################\n[i]\t:\tcont\t:\tperip");
                // Serial.print("\tmsg_len : ");
                // Serial.print(msg_len);
            #endif
            clear_message(controller_message);
            clear_message(peripheral_message);
            peripheral_message[spi_data_idx::length_idx] = msg_len;
            peripheral_message[spi_data_idx::send_cmd_idx] = spi_cmd::send_config::id; //dummy value to be traded when controller sends cmd
            peripheral_message[spi_data_idx::recv_cmd_idx] = 0; //dummy value to be traded when controller sends cmd
            
            uint8_t debug_location = 0;
            
            uint8_t cmd = 0;
            
            
            uint8_t i = 0;// send idx
            uint8_t j = 0;// recv idx
            while (my_spi.active() ) 
            {
                if (my_spi.available()) 
                {
                
                    if ( i >= sizeof(peripheral_message) ) // recieving data beyond message size likely due to error
                    {
                        //my_spi.pushr(0);
                    }
                    else// Still normally operating keep putting in data
                    {
                        //my_spi.pushr(utils::ff_to_fe(peripheral_message[i]));//converts any FF to FE before sending as the message appears as FF on the other side when there is an error
                        my_spi.pushr(peripheral_message[i]);
                    }
                
                    controller_message[j] = my_spi.popr();//pull the message off the buffer
                    // this method can potentially introduce errors if the j is not aligned with i
                    if(spi_data_idx::send_cmd_idx==j)
                    {
                        cmd = controller_message[j];
                        if (cmd != spi_cmd::send_config::id)
                        {
                            pack_data(peripheral_message, cmd, config_to_send, data, msg_len); 
                            debug_location = utils::update_bit(debug_location, 1, debug_send_data_bit);
                        }
                        else
                        {
                            pack_config(peripheral_message, config_to_send, msg_len);
                            debug_location = utils::update_bit(debug_location, 1, debug_send_config_bit);
                        }
                    }
                
                    i++;
                    
                    if((0xFF!=controller_message[j]&&0!=j)||(0==j && (/*0xFF!=controller_message[j]&&*/0!=controller_message[j])))//remove leading 0s and ignore FF as the other side sends this back if it was received
                    //if((0!=j)||(0==j && 0!=controller_message[j]))//remove leading 0s and ignore FF as the other side sends this back if it was received
                    {
                        j++;
                    }
                    #ifdef SPI_DEBUG_RAW    
                        // else if(0xFF==controller_message[j])
                        // {
                           // Serial.println("0xFF");
                        // }
                        // else if(0x0==controller_message[j])
                        // {
                           // Serial.println("0x0");
                        // }
                    #endif
                //is_unread_message = true;
                }
            }
        
            parse_message(controller_message, cmd, data);
            #ifdef SPI_DEBUG_RAW
                Serial.println("\n\nstatic_spi_handler::peripheral_transaction : \n#######################\n[i]\t:\tcont\t:\tperip");
                for( unsigned int i = 0; i<sizeof(peripheral_message); i++)
                {
                    Serial.print("[");
                    Serial.print(i);
                    Serial.print("]\t:\t");
                    Serial.print(controller_message[i]);
                    Serial.print("\t:\t");
                    Serial.println(peripheral_message[i]);
                }
                
            #endif
            return debug_location;
        };
    
        void pack_float(uint8_t* controller_message, uint8_t start_idx, float val)
        {
            uint8_t buff[sizeof(SPI_DATA_TYPE)];  // I don't like this but it was easier than trying to get to the right spot in the message;
            // #if SPI_DATA_TYPE == short int
                utils::float_to_short_fixed_point_bytes(val,buff,FIXED_POINT_FACTOR);
            // #elif SPI_DATA_TYPE == float
                // utils::float_to_uint8(val,buff);
            // #endif
            for(unsigned int i = 0; i<(sizeof(SPI_DATA_TYPE)); i++)
            {
                controller_message[start_idx + i] = ff_to_fe_check(buff[i]);
            }
        };
        
        float unpack_float(uint8_t* controller_message, uint8_t start_idx)
        {
            uint8_t buff[sizeof(SPI_DATA_TYPE)];  // I don't like this but it was easier than trying to get to the right spot in the message;
            float val;
            for(unsigned int i = 0; i<(sizeof(SPI_DATA_TYPE)); i++)
            {
                buff[i] = fe_to_ff_check(controller_message[start_idx + i]);
            }
            // #if SPI_DATA_TYPE == short int
                utils::short_fixed_point_bytes_to_float(buff, &val,FIXED_POINT_FACTOR);
            // #elif SPI_DATA_TYPE == float
                // utils::uint8_to_float(buff, &val);
            // #endif
            return val;
        };
        
        void pack_config(uint8_t* peripheral_message, uint8_t* config_to_send, uint8_t msg_len)
        {
            spi_data_idx::is_ff::clear_is_ff();
            peripheral_message[spi_data_idx::length_idx] = ff_to_fe_check(msg_len);
            peripheral_message[spi_data_idx::send_cmd_idx] = ff_to_fe_check(spi_cmd::send_config::id); //offsets value so data starts after cmd
            peripheral_message[spi_data_idx::recv_cmd_idx] = ff_to_fe_check(spi_cmd::send_config::id); //offsets value so data starts after cmd
            for(unsigned int i = 0; i<ini_config::number_of_keys; i++)
            {
                peripheral_message[i+spi_data_idx::base_idx_cnt+spi_data_idx::is_ff::num_bytes] = ff_to_fe_check(config_to_send[i]);
            }
            for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
            {
                peripheral_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
            }
            for(unsigned int i = spi_data_idx::base_idx_cnt + ini_config::number_of_keys + spi_data_idx::is_ff::num_bytes; i<msg_len ; i++)
            {
                peripheral_message[i] = 0;
            }
        };
        
        void pack_data(uint8_t* peripheral_message, uint8_t cmd, uint8_t* config_to_send, ExoData* data, uint8_t msg_len)
        {
            spi_data_idx::is_ff::clear_is_ff();
            
            uint8_t running_idx_cnt = 0;
            
            peripheral_message[running_idx_cnt + spi_data_idx::length_idx] = ff_to_fe_check(msg_len);
            peripheral_message[running_idx_cnt + spi_data_idx::send_cmd_idx] = ff_to_fe_check(spi_cmd::send_data_exo::id);
            peripheral_message[running_idx_cnt + spi_data_idx::recv_cmd_idx] = ff_to_fe_check(cmd);
            running_idx_cnt = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes;
            
            // send exo top level data  just uint8_t
            peripheral_message[running_idx_cnt + spi_data_idx::exo::exo_status] = ff_to_fe_check(data->status);
            peripheral_message[running_idx_cnt + spi_data_idx::exo::sync_led_state] = ff_to_fe_check(data->sync_led_state);
            running_idx_cnt += spi_data_idx::exo::idx_cnt;
            
            // left leg used    
            if (config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::bilateral || config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::left)
            {
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_heel_fsr] = ff_to_fe_check(data->left_leg.do_calibration_heel_fsr);
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_toe_fsr] = ff_to_fe_check(data->left_leg.do_calibration_toe_fsr);
                
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_heel_fsr] = ff_to_fe_check(data->left_leg.do_calibration_refinement_heel_fsr);
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_toe_fsr] = ff_to_fe_check(data->left_leg.do_calibration_refinement_toe_fsr);
                
                pack_float(peripheral_message, running_idx_cnt + spi_data_idx::leg::percent_gait, data->left_leg.percent_gait);
                pack_float(peripheral_message, running_idx_cnt + spi_data_idx::leg::heel_fsr, data->left_leg.heel_fsr);
                pack_float(peripheral_message, running_idx_cnt + spi_data_idx::leg::toe_fsr, data->left_leg.toe_fsr);
                running_idx_cnt += spi_data_idx::leg::idx_cnt;
                
                // left hip used
                if (config_to_send[config_defs::hip_idx] != (uint8_t)config_defs::motor::not_used)
                {
                    
                    peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor] = ff_to_fe_check(data->left_leg.hip.calibrate_torque_sensor);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::joint::torque, data->left_leg.hip.torque_reading);
                    running_idx_cnt += spi_data_idx::joint::idx_cnt;
                    
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p, data->left_leg.hip.motor.p);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v, data->left_leg.hip.motor.v);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::i, data->left_leg.hip.motor.i);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des, data->left_leg.hip.motor.p_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des, data->left_leg.hip.motor.v_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff, data->left_leg.hip.motor.t_ff);
                    running_idx_cnt += spi_data_idx::motor::idx_cnt;
                }
                
                // left knee used
                if (config_to_send[config_defs::knee_idx] != (uint8_t)config_defs::motor::not_used)
                {
                    peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor] = ff_to_fe_check(data->left_leg.knee.calibrate_torque_sensor);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::joint::torque, data->left_leg.knee.torque_reading);
                    running_idx_cnt += spi_data_idx::joint::idx_cnt;
                    
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p, data->left_leg.knee.motor.p);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v, data->left_leg.knee.motor.v);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::i, data->left_leg.knee.motor.i);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des, data->left_leg.knee.motor.p_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des, data->left_leg.knee.motor.v_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff, data->left_leg.knee.motor.t_ff);
                    running_idx_cnt += spi_data_idx::motor::idx_cnt;
                }
                
                // left ankle used
                if (config_to_send[config_defs::ankle_idx] != (uint8_t)config_defs::motor::not_used)
                {
                    peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor] = ff_to_fe_check(data->left_leg.ankle.calibrate_torque_sensor);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::joint::torque, data->left_leg.ankle.torque_reading);
                    running_idx_cnt += spi_data_idx::joint::idx_cnt;
                    
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p, data->left_leg.ankle.motor.p);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v, data->left_leg.ankle.motor.v);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::i, data->left_leg.ankle.motor.i);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des, data->left_leg.ankle.motor.p_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des, data->left_leg.ankle.motor.v_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff, data->left_leg.ankle.motor.t_ff);
                    running_idx_cnt += spi_data_idx::motor::idx_cnt;
                }
                
            }
            
            // right leg used
            if (config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::bilateral || config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::right)
            {
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_heel_fsr] = ff_to_fe_check(data->right_leg.do_calibration_heel_fsr);
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_toe_fsr] = ff_to_fe_check(data->right_leg.do_calibration_toe_fsr);
                
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_heel_fsr] = ff_to_fe_check(data->right_leg.do_calibration_refinement_heel_fsr);
                peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_toe_fsr] = ff_to_fe_check(data->right_leg.do_calibration_refinement_toe_fsr);
                
                pack_float(peripheral_message, running_idx_cnt + spi_data_idx::leg::percent_gait, data->right_leg.percent_gait);
                pack_float(peripheral_message, running_idx_cnt + spi_data_idx::leg::heel_fsr, data->right_leg.heel_fsr);
                pack_float(peripheral_message, running_idx_cnt + spi_data_idx::leg::toe_fsr, data->right_leg.toe_fsr);
                running_idx_cnt += spi_data_idx::leg::idx_cnt;
                
                // right hip used
                if (config_to_send[config_defs::hip_idx] != (uint8_t)config_defs::motor::not_used)
                {
                    peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor] = ff_to_fe_check(data->right_leg.hip.calibrate_torque_sensor);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::joint::torque, data->right_leg.hip.torque_reading);
                    running_idx_cnt += spi_data_idx::joint::idx_cnt;
                    
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p, data->right_leg.hip.motor.p);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v, data->right_leg.hip.motor.v);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::i, data->right_leg.hip.motor.i);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des, data->right_leg.hip.motor.p_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des, data->right_leg.hip.motor.v_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff, data->right_leg.hip.motor.t_ff);
                    running_idx_cnt += spi_data_idx::motor::idx_cnt;
                }
                
                // right knee used
                if (config_to_send[config_defs::knee_idx] != (uint8_t)config_defs::motor::not_used)
                {
                    peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor] = ff_to_fe_check(data->right_leg.knee.calibrate_torque_sensor);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::joint::torque, data->right_leg.knee.torque_reading);
                    running_idx_cnt += spi_data_idx::joint::idx_cnt;
                    
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p, data->right_leg.knee.motor.p);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v, data->right_leg.knee.motor.v);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::i, data->right_leg.knee.motor.i);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des, data->right_leg.knee.motor.p_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des, data->right_leg.knee.motor.v_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff, data->right_leg.knee.motor.t_ff);
                    running_idx_cnt += spi_data_idx::motor::idx_cnt;
                }
                
                // right ankle used
                if (config_to_send[config_defs::ankle_idx] != (uint8_t)config_defs::motor::not_used)
                {
                    peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor] = ff_to_fe_check(data->right_leg.ankle.calibrate_torque_sensor);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::joint::torque, data->right_leg.ankle.torque_reading);
                    running_idx_cnt += spi_data_idx::joint::idx_cnt;
                    
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p, data->right_leg.ankle.motor.p);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v, data->right_leg.ankle.motor.v);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::i, data->right_leg.ankle.motor.i);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des, data->right_leg.ankle.motor.p_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des, data->right_leg.ankle.motor.v_des);
                    pack_float(peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff, data->right_leg.ankle.motor.t_ff);
                    running_idx_cnt += spi_data_idx::motor::idx_cnt;
                }
            }
            
            for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
            {
                peripheral_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
            }
            for(unsigned int i = running_idx_cnt; i<msg_len ; i++)
            {
                peripheral_message[i] = 0;
            }
        };
        
        void parse_message(uint8_t* controller_message, uint8_t command, ExoData* data)
        {
            #ifdef SPI_DEBUG
                Serial.print("\n\n");
            #endif
            spi_data_idx::is_ff::clear_is_ff();
            for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
            {
                spi_data_idx::is_ff::is_ff[i] = controller_message[i+spi_data_idx::base_idx_cnt];
                #ifdef SPI_DEBUG
                    Serial.print("static_spi_handler::parse_message : is_ff[");
                    Serial.print(i);
                    Serial.print("] = 0b");
                    Serial.println(spi_data_idx::is_ff::is_ff[i],BIN);
                #endif
            }
            #ifdef SPI_DEBUG
                Serial.print("static_spi_handler::parse_message : ");
            #endif   

            uint8_t joint;
            switch (command)
            {
                case spi_cmd::send_data_exo::id:
                    // nothing to parse
                    #ifdef SPI_DEBUG
                        Serial.println("send_data_exo");
                    #endif
                    break;
                case spi_cmd::send_config::id:
                    // nothing to parse
                    #ifdef SPI_DEBUG
                        Serial.println("send_config");
                    #endif
                    break;
                case spi_cmd::null_cmd::id:
                    // nothing to parse
                    #ifdef SPI_DEBUG
                        Serial.println("null_cmd");
                    #endif
                    break;
                case spi_cmd::update_controller::id:
                    
                    #ifdef SPI_DEBUG
                        Serial.print("update_controller : ");
                    #endif
                    joint = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::joint_id_idx]);
                    switch (joint)
                    {
                        // left
                        case (uint8_t)config_defs::joint_id::left_hip:
                            data->left_leg.hip.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Left Hip : ");
                                Serial.println(data->left_leg.hip.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_knee:
                            data->left_leg.knee.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Left Knee : ");
                                Serial.println(data->left_leg.knee.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_ankle:
                            data->left_leg.ankle.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Left Ankle : ");
                                Serial.println(data->left_leg.ankle.controller.controller);
                            #endif
                            break;
                        // right
                        case (uint8_t)config_defs::joint_id::right_hip:
                            data->right_leg.hip.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Right Hip : ");
                                Serial.println(data->right_leg.hip.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_knee:
                            data->right_leg.knee.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Right Knee : ");
                                Serial.println(data->right_leg.knee.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_ankle:
                            data->right_leg.ankle.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Right Ankle : ");
                                Serial.println(data->right_leg.ankle.controller.controller);
                            #endif
                            break;
                    }
                    break;
                case spi_cmd::update_controller_params::id:
                    #ifdef SPI_DEBUG
                        Serial.print("update_controller_params : ");
                    #endif
                    joint = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::joint_id_idx]);
                    switch (joint)
                    {
                        // left
                        case (uint8_t)config_defs::joint_id::left_hip:
                            data->left_leg.hip.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.println("Left Hip : Controller");
                                Serial.println(data->left_leg.hip.controller.controller);
                            #endif
                            for (int message_idx = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                data->left_leg.hip.controller.parameters[parameter_idx] = unpack_float(controller_message, message_idx);
                                #ifdef SPI_DEBUG
                                    Serial.print("[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] :\t");
                                    Serial.println(data->left_leg.hip.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::left_knee:
                            data->left_leg.knee.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.println("Left Knee : Controller");
                                Serial.println(data->left_leg.knee.controller.controller);
                            #endif
                            for (int message_idx = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                data->left_leg.knee.controller.parameters[parameter_idx] = unpack_float(controller_message, message_idx);
                                #ifdef SPI_DEBUG
                                    Serial.print("[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] :\t");
                                    Serial.println(data->left_leg.knee.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::left_ankle:
                            data->left_leg.ankle.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.println("Left Ankle : Controller");
                                Serial.println(data->left_leg.ankle.controller.controller);
                            #endif
                            for (int message_idx = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                data->left_leg.ankle.controller.parameters[parameter_idx] = unpack_float(controller_message, message_idx);
                                #ifdef SPI_DEBUG
                                    Serial.print("[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] :\t");
                                    Serial.println(data->left_leg.ankle.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        // right
                        case (uint8_t)config_defs::joint_id::right_hip:
                            data->right_leg.hip.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.println("Right Hip : Controller");
                                Serial.println(data->right_leg.hip.controller.controller);
                            #endif
                            for (int message_idx = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                data->right_leg.hip.controller.parameters[parameter_idx] = unpack_float(controller_message, message_idx);
                                #ifdef SPI_DEBUG
                                    Serial.print("[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] :\t");
                                    Serial.println(data->right_leg.hip.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::right_knee:
                            data->right_leg.knee.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.println("Right Knee : Controller");
                                Serial.println(data->right_leg.knee.controller.controller);
                            #endif
                            for (int message_idx = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                data->right_leg.knee.controller.parameters[parameter_idx] = unpack_float(controller_message, message_idx);
                            
                                #ifdef SPI_DEBUG
                                    Serial.print("[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] :\t");
                                    Serial.println(data->right_leg.knee.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::right_ankle:
                            data->right_leg.ankle.controller.controller = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::controller_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Right Ankle : Controller");
                                Serial.println(data->right_leg.ankle.controller.controller);
                            #endif
                            for (int message_idx = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                data->right_leg.ankle.controller.parameters[parameter_idx] = unpack_float(controller_message, message_idx);
                            
                                #ifdef SPI_DEBUG
                                    Serial.print("[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] :\t");
                                    Serial.println(data->right_leg.ankle.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                    }
                    break;
                case spi_cmd::calibrate_torque_sensor::id:
                    #ifdef SPI_DEBUG
                        Serial.print("calibrate_torque_sensor : ");
                    #endif
                    
                    joint = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::joint_id_idx]);
                    switch (joint)
                    {
                        // left
                        case (uint8_t)config_defs::joint_id::left_hip:
                            data->left_leg.hip.calibrate_torque_sensor = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Left Hip : ");
                                Serial.println(data->left_leg.hip.calibrate_torque_sensor);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_knee:
                            data->left_leg.knee.calibrate_torque_sensor = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Left Knee : ");
                                Serial.println(data->left_leg.knee.calibrate_torque_sensor);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_ankle:
                            data->left_leg.ankle.calibrate_torque_sensor = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Left Ankle : ");
                                Serial.println(data->left_leg.ankle.calibrate_torque_sensor);
                            #endif
                            break;
                        // right
                        case (uint8_t)config_defs::joint_id::right_hip:
                            data->right_leg.hip.calibrate_torque_sensor = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Right Hip : ");
                                Serial.println(data->right_leg.hip.calibrate_torque_sensor);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_knee:
                            data->right_leg.knee.calibrate_torque_sensor = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Right Knee : ");
                                Serial.println(data->right_leg.knee.calibrate_torque_sensor);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_ankle:
                            data->right_leg.ankle.calibrate_torque_sensor = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx]);
                            #ifdef SPI_DEBUG
                                Serial.print("Right Ankle : ");
                                Serial.println(data->right_leg.ankle.calibrate_torque_sensor);
                            #endif
                            break;
                    }
                    break;
                case spi_cmd::calibrate_fsr::id:
                    data->left_leg.do_calibration_heel_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::left_heel_idx]);
                    data->left_leg.do_calibration_toe_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::left_toe_idx]);
                    data->right_leg.do_calibration_heel_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::right_heel_idx]);
                    data->right_leg.do_calibration_toe_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::right_toe_idx]);
                    #ifdef SPI_DEBUG
                        Serial.println("calibrate_fsr");
                        Serial.print("\t");
                        Serial.print(data->left_leg.do_calibration_heel_fsr);
                        Serial.print("\t");
                        Serial.print(data->left_leg.do_calibration_toe_fsr);
                        Serial.print("\t");
                        Serial.print(data->right_leg.do_calibration_heel_fsr);
                        Serial.print("\t");
                        Serial.println(data->right_leg.do_calibration_toe_fsr);
                    #endif
                    break;
                case spi_cmd::refine_fsr::id:
                    data->left_leg.do_calibration_refinement_heel_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::left_heel_idx]);
                    data->left_leg.do_calibration_refinement_toe_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::left_toe_idx]);
                    data->right_leg.do_calibration_refinement_heel_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::right_heel_idx]);
                    data->right_leg.do_calibration_refinement_toe_fsr = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::right_toe_idx]);
                    #ifdef SPI_DEBUG
                        Serial.println("refine_fsr");
                        Serial.print("\t");
                        Serial.print(data->left_leg.do_calibration_refinement_heel_fsr);
                        Serial.print("\t");
                        Serial.print(data->left_leg.do_calibration_refinement_toe_fsr);
                        Serial.print("\t");
                        Serial.print(data->right_leg.do_calibration_refinement_heel_fsr);
                        Serial.print("\t");
                        Serial.println(data->right_leg.do_calibration_refinement_toe_fsr);
                    #endif
                    break;
                case spi_cmd::motor_enable_disable::id:
                    data->left_leg.hip.motor.enabled = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_hip_idx]);
                    data->left_leg.knee.motor.enabled = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_knee_idx]);
                    data->left_leg.ankle.motor.enabled = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_ankle_idx]);
                    data->right_leg.hip.motor.enabled = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_hip_idx]);
                    data->right_leg.knee.motor.enabled = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_knee_idx]);
                    data->right_leg.ankle.motor.enabled = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_ankle_idx]);
                    #ifdef SPI_DEBUG
                        Serial.println("motor_enable_disable");
                        Serial.print("\t");
                        Serial.print(data->left_leg.hip.motor.enabled);
                        Serial.print("\t");
                        Serial.print(data->left_leg.knee.motor.enabled);
                        Serial.print("\t");
                        Serial.print(data->left_leg.ankle.motor.enabled);
                        Serial.print("\t");
                        Serial.print(data->right_leg.hip.motor.enabled);
                        Serial.print("\t");
                        Serial.print(data->right_leg.knee.motor.enabled);
                        Serial.print("\t");
                        Serial.println(data->right_leg.ankle.motor.enabled);
                    #endif
                    break;
                case spi_cmd::motor_zero::id:
                    data->left_leg.hip.motor.do_zero = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_hip_idx]);
                    data->left_leg.knee.motor.do_zero = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_knee_idx]);
                    data->left_leg.ankle.motor.do_zero = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_ankle_idx]);
                    data->right_leg.hip.motor.do_zero = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_hip_idx]);
                    data->right_leg.knee.motor.do_zero = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_knee_idx]);
                    data->right_leg.ankle.motor.do_zero = fe_to_ff_check(controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_ankle_idx]);
                    #ifdef SPI_DEBUG
                        Serial.println("motor_zero");
                        Serial.print("\t");
                        Serial.print(data->left_leg.hip.motor.do_zero);
                        Serial.print("\t");
                        Serial.print(data->left_leg.knee.motor.do_zero);
                        Serial.print("\t");
                        Serial.print(data->left_leg.ankle.motor.do_zero);
                        Serial.print("\t");
                        Serial.print(data->right_leg.hip.motor.do_zero);
                        Serial.print("\t");
                        Serial.print(data->right_leg.knee.motor.do_zero);
                        Serial.print("\t");
                        Serial.println(data->right_leg.ankle.motor.do_zero);
                    #endif
                    
                    break;
                // case spi_cmd::::id:
                    
                    // break;               
            }
        };
        
        void clear_message(uint8_t* message)
        {
            for(unsigned int i = 0; i<sizeof(message); i++)
            {
                message[i] = 0;
            }
        };
        
        void print_message(uint8_t* controller_message, ExoData* data, uint8_t command)
        {
            
            #ifdef SPI_DEBUG
                Serial.print("SPIHandler::print_message() : command : ");
                print_message_name(command);
                Serial.println("");
                Serial.println("SPIHandler::print_message() : controller_message : ");
                for (int i = 0 ; i<spi_cmd::max_param_len ; i++)
                {
                    if(0!=controller_message[i])
                    {
                        Serial.print("\t[");
                        Serial.print(i);
                        Serial.print("] = ");
                        Serial.println(controller_message[i]);
                    }
                }
            #endif
            switch (command)
            {
                case spi_cmd::send_data_exo::id:
                    // nothing to parse
                    #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::_parse_message() : send_data_exo : ");
                        Serial.println(command);
                        Serial.println("SPIHandler::_parse_message() : controller_message : ");
                        for (int i = 0 ; i<spi_cmd::max_param_len ; i++)
                        {
                            if(0!=controller_message[i])
                            {
                                Serial.print("\t[");
                                Serial.print(i);
                                Serial.print("] = ");
                                Serial.println(controller_message[i]);
                            }
                        }
                    #endif
                    break;
                case spi_cmd::send_config::id:
                    // nothing to parse
                    #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::print_message() : send_config : ");
                        Serial.println(command);
                        Serial.println("SPIHandler::print_message() : controller_message : ");
                        for (int i = 0 ; i<spi_cmd::max_param_len ; i++)
                        {
                            if(0!=controller_message[i])
                            {
                                Serial.print("\t[");
                                Serial.print(i);
                                Serial.print("] = ");
                                Serial.println(controller_message[i]);
                            }
                        }
                    #endif
                    break;
                case spi_cmd::null_cmd::id:
                    // nothing to parse
                    #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::print_message() : null_cmd : ");
                        Serial.println(command);
                        Serial.println("SPIHandler::print_message() : controller_message : ");
                        for (int i = 0 ; i<spi_cmd::max_param_len ; i++)
                        {
                            if(0!=controller_message[i])
                            {
                                Serial.print("\t[");
                                Serial.print(i);
                                Serial.print("] = ");
                                Serial.println(controller_message[i]);
                            }
                        }
                    #endif
                    break;
                case spi_cmd::update_controller::id:
                    switch (controller_message[spi_cmd::update_controller::joint_id_idx])
                    {
                        #ifdef SPI_DEBUG
                            Serial.println("SPIHandler::print_message() : update_controller");
                        #endif
                        // left
                        case (uint8_t)config_defs::joint_id::left_hip:
                            #ifdef SPI_DEBUG
                                Serial.print("\tLeft Hip : Controller = ");
                                Serial.print(data->left_leg.hip.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_knee:
                            #ifdef SPI_DEBUG
                                Serial.print("\tLeft Knee : Controller = ");
                                Serial.print(data->left_leg.knee.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_ankle:
                            #ifdef SPI_DEBUG
                                Serial.print("\tLeft Ankle : Controller = ");
                                Serial.print(data->left_leg.ankle.controller.controller);
                            #endif
                            break;
                        // right
                        case (uint8_t)config_defs::joint_id::right_hip:
                            #ifdef SPI_DEBUG
                                Serial.print("\tRight Hip : Controller = ");
                                Serial.print(data->right_leg.hip.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_knee:
                            #ifdef SPI_DEBUG
                                Serial.print("\tRight Knee : Controller = ");
                                Serial.print(data->right_leg.knee.controller.controller);
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_ankle:
                            #ifdef SPI_DEBUG
                                Serial.print("\tRight Ankle : Controller = ");
                                Serial.print(data->right_leg.ankle.controller.controller);
                            #endif
                            break;
                    }
                    break;
                case spi_cmd::update_controller_params::id:
                    switch (controller_message[spi_cmd::update_controller_params::joint_id_idx])
                    {
                        #ifdef SPI_DEBUG
                            Serial.println("SPIHandler::_parse_message() : update_controller_params");
                        #endif
                        // left
                        case (uint8_t)config_defs::joint_id::left_hip:
                            for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                #ifdef SPI_DEBUG
                                    Serial.print("\tLeft Hip : Parameter[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] = ");
                                    Serial.println(data->left_leg.hip.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::left_knee:
                            for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                #ifdef SPI_DEBUG
                                    Serial.print("\tLeft Knee : Parameter[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] = ");
                                    Serial.println(data->left_leg.knee.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::left_ankle:
                            for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                #ifdef SPI_DEBUG
                                    Serial.print("\tLeft Ankle : Parameter[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] = ");
                                    Serial.println(data->left_leg.ankle.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        // right
                        case (uint8_t)config_defs::joint_id::right_hip:
                            for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                #ifdef SPI_DEBUG
                                    Serial.print("\tRight Hip : Parameter[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] = ");
                                    Serial.println(data->right_leg.hip.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::right_knee:
                            for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                #ifdef SPI_DEBUG
                                    Serial.print("\tRight Knee : Parameter[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] = ");
                                    Serial.println(data->right_leg.knee.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                        case (uint8_t)config_defs::joint_id::right_ankle:
                            for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                            {
                                #ifdef SPI_DEBUG
                                    Serial.print("\tRight Ankle : Parameter[");
                                    Serial.print(parameter_idx);
                                    Serial.print("] = ");
                                    Serial.println(data->right_leg.ankle.controller.parameters[parameter_idx]);
                                #endif
                            }
                            break;
                    }
                    break;
                case spi_cmd::calibrate_torque_sensor::id:
                    switch (controller_message[spi_cmd::calibrate_torque_sensor::joint_id_idx])
                    {
                        // left
                        case (uint8_t)config_defs::joint_id::left_hip:
                            #ifdef SPI_DEBUG
                                Serial.println("SPIHandler::_parse_message() : calibrate_torque_sensor - Left Hip");
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_knee:
                            #ifdef SPI_DEBUG
                                Serial.println("SPIHandler::_parse_message() : calibrate_torque_sensor - Left Knee");
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::left_ankle:
                            #ifdef SPI_DEBUG
                                Serial.println("SPIHandler::_parse_message() : calibrate_torque_sensor - Left Ankle");
                            #endif
                            break;
                        // right
                        case (uint8_t)config_defs::joint_id::right_hip:
                            #ifdef SPI_DEBUG
                                Serial.println("SPIHandler::_parse_message() : calibrate_torque_sensor - Right Hip");
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_knee:
                            #ifdef SPI_DEBUG
                                Serial.println("SPIHandler::_parse_message() : calibrate_torque_sensor - Right Knee");
                            #endif
                            break;
                        case (uint8_t)config_defs::joint_id::right_ankle:
                            #ifdef SPI_DEBUG
                                Serial.println("SPIHandler::_parse_message() : calibrate_torque_sensor - Right Ankle");
                            #endif
                            break;
                    }
                    break;
                case spi_cmd::calibrate_fsr::id:
                    #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::_parse_message() : calibrate_fsr - ");
                        Serial.print(data->left_leg.do_calibration_heel_fsr);
                        Serial.print(data->left_leg.do_calibration_toe_fsr);
                        Serial.print(data->right_leg.do_calibration_heel_fsr);
                        Serial.println(data->right_leg.do_calibration_toe_fsr);
                    #endif
                    break;
                case spi_cmd::refine_fsr::id:
                    #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::_parse_message() : refine_fsr - ");
                        Serial.print(data->left_leg.do_calibration_refinement_heel_fsr);
                        Serial.print(data->left_leg.do_calibration_refinement_heel_fsr);
                        Serial.print(data->right_leg.do_calibration_refinement_heel_fsr);
                        Serial.println(data->right_leg.do_calibration_refinement_heel_fsr);
                    #endif
                    break;
                case spi_cmd::motor_enable_disable::id:
                    #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::_parse_message() : motor_enable_disable - ");
                        Serial.print(data->left_leg.hip.motor.enabled);
                        Serial.print(data->left_leg.knee.motor.enabled);
                        Serial.print(data->left_leg.ankle.motor.enabled);
                        Serial.print(data->right_leg.hip.motor.enabled);
                        Serial.print(data->right_leg.knee.motor.enabled);
                        Serial.println(data->right_leg.ankle.motor.enabled);
                    #endif
                    break;
                case spi_cmd::motor_zero::id:
                   #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::_parse_message() : motor_zero - ");
                        Serial.print(data->left_leg.hip.motor.do_zero);
                        Serial.print(data->left_leg.knee.motor.do_zero);
                        Serial.print(data->left_leg.ankle.motor.do_zero);
                        Serial.print(data->right_leg.hip.motor.do_zero);
                        Serial.print(data->right_leg.knee.motor.do_zero);
                        Serial.println(data->right_leg.ankle.motor.do_zero);
                    #endif
                    break;
                // case spi_cmd::::id:
                    
                    // break;  
                default :
                    #ifdef SPI_DEBUG
                        Serial.print("SPIHandler::print_message() : Unknown Command : ");
                        Serial.println(command);
                        Serial.println("SPIHandler::print_message() : controller_message : ");
                        for (int i = 0 ; i<spi_cmd::max_param_len ; i++)
                        {
                            Serial.print("\t[");
                            Serial.print(i);
                            Serial.print("] = ");
                            Serial.println(controller_message[i]);
                        }
                    #endif
                    break;    
            }
        };
    
        void set_message_flag(bool* is_unread_message)
        {
            *is_unread_message = true;
        }
        
        void print_debug(uint16_t debug_location)
        {
            if(utils::get_bit(debug_location,debug_entry_bit))
            {
                Serial.println("static_spi_handler::print_debug : Entered");
            }
            if(utils::get_bit(debug_location,debug_sent_length_bit))
            {
                Serial.println("static_spi_handler::print_debug : Sent Length");
            }
            if(utils::get_bit(debug_location,debug_send_config_bit))
            {
                Serial.println("static_spi_handler::print_debug : Sent Config");
            }
            if(utils::get_bit(debug_location,debug_send_data_bit))
            {
                Serial.println("static_spi_handler::print_debug : Sent Data");
            }
            
            
            
            
            if(utils::get_bit(debug_location,debug_exit_bit))
            {
                Serial.println("static_spi_handler::print_debug : Exited");
            }
        }
       
    }    
    
    
    

#elif defined(ARDUINO_ARDUINO_NANO33BLE)
    #include <SPI.h>
    
    // ===========================================================================================================  
    SPIHandler::SPIHandler(uint8_t* config_to_send, ExoData* exo_data)
    //: _config_to_send(config_to_send) // initalize the const pointer to config_to_send as the SPI was breaking it.
    {
        pinMode(coms_micro_pins::cs_pin, OUTPUT);
        digitalWrite(coms_micro_pins::cs_pin, HIGH);
        SPI.begin();
        //uintptr_t  temp_config_address = (uintptr_t)&config_to_send[0];
        // set the local variable
        _data = exo_data;
        _config_to_send = config_to_send; 
        
        // clear the message just in case
        _clear_message();
        _msg_len = sizeof(_controller_message);
        
        #ifdef SPI_DEBUG
            Serial.print("SPIHandler::SPIHandler : _data pointing address: ");
            Serial.println((long)(_data), HEX);
            Serial.print("SPIHandler::SPIHandler : _config_to_send address at start: ");
            Serial.println((long)&_config_to_send[0], HEX);
            // Serial.print("SPIHandler::SPIHandler : temp_config_address: ");
            // Serial.println((long)temp_config_address, HEX);
            Serial.println("SPIHandler::SPIHandler : _config_to_send");
        #endif
        for(int i = 0; i<ini_config::number_of_keys; i++)
        {
            //_config_to_send[i] = config_to_send[i];
            #ifdef SPI_DEBUG
                Serial.print("\t");
                Serial.println(_config_to_send[i]);
            #endif
        }
        
        #ifdef SPI_DEBUG
            Serial.print("SPIHandler::SPIHandler : _data pointing address: ");
            Serial.println((long)(_data), HEX);
            Serial.print("SPIHandler::SPIHandler : _config_to_send address after _data_len: ");
            Serial.println((long)&_config_to_send[0], HEX);
        #endif
        
        #ifdef SPI_DEBUG
            Serial.print("SPIHandler::SPIHandler : _data pointing address: ");
            Serial.println((long)(_data), HEX);
            Serial.print("SPIHandler::SPIHandler : _config_to_send address after _clear_message : ");
            Serial.println((long)&_config_to_send[0], HEX);
            // Serial.print("SPIHandler::SPIHandler : temp_config_address: ");
            // Serial.println(temp_config_address, HEX);
            //_config_to_send = (uint8_t*)temp_config_address;
            // Serial.print("_config_to_send address after reset: ");
            // Serial.println((long)&_config_to_send[0], HEX);
        #endif
        //_my_spi = my_spi;
        _command = spi_cmd::send_config::id;
        
        #ifdef SPI_DEBUG
            Serial.print("SPIHandler::SPIHandler : _data pointing address: ");
            Serial.println((long)(_data), HEX);
            Serial.print("SPIHandler::SPIHandler : _config_to_send address after _command: ");
            Serial.println((long)&_config_to_send[0], HEX);
        #endif
        transaction(_command);
        // TODO: Figure out why send config to other message introduces error on teensy first byte.
        _command = spi_cmd::send_data_exo::id;
        transaction(_command);
        transaction(_command);
        // _pack_message();
        // _send_message();
        // _parse_message();
        // _pack_message();
        // _send_message();// send the request for config again as the messages were getting messed up
        
        #ifdef SPI_DEBUG_RAW
            // Serial.println("\n\nSPIHandler::SPIHandler : \n#######################\n[i]\t:\tcont\t:\tperip");
            // for( int i = 0; i<_msg_len; i++)
            // {
                // Serial.print("[");
                // Serial.print(i);
                // Serial.print("]\t:\t");
                // Serial.print(_controller_message[i]);
                // Serial.print("\t:\t");
                // Serial.println(_peripheral_message[i]);
            // }
        #endif
        _clear_message();
    };
    
    void SPIHandler::transaction(uint8_t command)
    {
        #ifdef SPI_DEBUG
            Serial.println("\n\nSPIHandler::transaction : Entered");
        #endif
        // the coms micro receives the sent message delayed by 1 transaction.
        // messages from the coms micro alternate command (COMS will read the data_length on that transaction, and logic micro will set the tx buffer with config or exo_data)
        // and info associated with the last command (COMS will read the config or exo_data, and logic micro will set the tx buffer with buffer length).
        uint8_t possible_data_len;
        // config not set yet, keep trying till it gets it.
        
       
        
        #ifdef SPI_DEBUG
            // Serial.print("SPIHandler::transaction : _config_to_send address : ");
            // Serial.println((long)&_config_to_send[0], HEX);
            
            // Serial.println("SPIHandler::transaction : _config_to_send at start : ");
            // for(int i = 0; i<ini_config::number_of_keys; i++)
            // {
                // //_config_to_send[i] = ini_config::number_of_keys-i;
                // Serial.print("\t_config_to_send = ");
                // Serial.println(_config_to_send[i]);
            // }
        #endif
        // while (_config_to_send[0] == 0)
        // {
            // _command = spi_cmd::send_config::id;
            // possible_data_len = _send_command();
            // _pack_message();
            // _send_message();
            // _parse_message();
            // Serial.println("SPIHandler::transaction : waiting for config");
            
        // }
        
        _command = command;
        _pack_message();
        #ifdef SPI_DEBUG_RAW
            // Serial.println("\n\SPIHandler::transaction (pre parse) : \n#######################\n[i]\t:\tcont\t:\tperip");
            // Serial.print("\t_msg_len : ");
            // Serial.print(_msg_len);
        #endif
        // if the data_lengths match we are ok and the command and message pairs are aligned or if we are asking for the config the data length is likely not set so we need to read as normal to set it.
        _send_message();
        #ifdef SPI_DEBUG
            Serial.println("SPIHandler::transaction : message sent");
        #endif
        _parse_message();
        #ifdef SPI_DEBUG_RAW
            // Serial.println("\n\nSPIHandler::transaction : \n[i]\t:\tcont\t:\tperip");
            // Serial.print("\t_msg_len : ");
            // Serial.print(_msg_len);
        #endif
        #ifdef SPI_DEBUG
            Serial.println("SPIHandler::transaction (post parse) : message parsed");
        #endif
        #ifdef SPI_DEBUG_RAW
            Serial.println("\n\nSPIHandler::transaction : \n#######################\n[i]\t:\tcont\t:\tperip");
            for( int i = 0; i<_msg_len; i++)
            {
                Serial.print("[");
                Serial.print(i);
                Serial.print("]\t:\t");
                Serial.print(_controller_message[i]);
                Serial.print("\t:\t");
                Serial.println(_peripheral_message[i]);
            }
        #endif
        _clear_message();
        return;
    };
      
    void SPIHandler::transaction(uint8_t command, uint8_t joint_id)
    {
        _joint_id = joint_id;
        transaction(command);
        // clear the joint id so we don't do something accidently 
        _joint_id = 0;
    };
    
    void SPIHandler::_pack_float(uint8_t* message, uint8_t start_idx, float val)
    {
        uint8_t buff[sizeof(SPI_DATA_TYPE)];  // I don't like this but it was easier than trying to get to the right spot in the message;
        // #if SPI_DATA_TYPE == short int
            utils::float_to_short_fixed_point_bytes(val,buff,FIXED_POINT_FACTOR);
        // #elif SPI_DATA_TYPE == float
            // utils::float_to_uint8(val,buff);
        // #endif
        for(int i = 0; i<(sizeof(SPI_DATA_TYPE)); i++)
        {
            message[start_idx + i] = ff_to_fe_check(buff[i]);
        }
    };
    
    float SPIHandler::_unpack_float(uint8_t* message, uint8_t start_idx)
    {
        uint8_t buff[sizeof(SPI_DATA_TYPE)];  // I don't like this but it was easier than trying to get to the right spot in the message;
        float val;
        #ifdef SPI_DEBUG
            // Serial.println("SPIHandler::_unpack_float : ");
        #endif
        for(int i = 0; i<(sizeof(SPI_DATA_TYPE)); i++)
        {
            #ifdef SPI_DEBUG
                // Serial.print("\t[");
                // Serial.print(i);
                // Serial.print("] : ");
                // Serial.print(message[start_idx + i]);
                // Serial.print(" -> ");
            #endif
            buff[i] = fe_to_ff_check(message[start_idx + i]);
            #ifdef SPI_DEBUG
                // Serial.println(buff[i]);
            #endif
        }
        // #if SPI_DATA_TYPE == short int
            utils::short_fixed_point_bytes_to_float(buff, &val,FIXED_POINT_FACTOR);
        // #elif SPI_DATA_TYPE == float
            // utils::uint8_to_float(buff, &val);
        // #endif
        
        #ifdef SPI_DEBUG
            // Serial.print("\t");
            // Serial.println(val);
        #endif
        return val;
    };
    
    void SPIHandler::_parse_message()
    {
        #ifdef SPI_DEBUG
            Serial.print("SPIHandler::_parse_message : _command : ");
            print_message_name(_command);
            Serial.println();
            // Serial.print("SPIHandler::_parse_message : _command address: 0x");
            // Serial.println((long)&_command,HEX);
            
        #endif
        switch (_command)
        {
            case spi_cmd::send_config::id:
                _parse_config();
                break;
            default:
                _parse_data();
                break;     
        }
    };
    
    void SPIHandler::_parse_config()
    {
        #ifdef SPI_DEBUG
            Serial.println("SPIHandler::_parse_config : Entered");
            // Serial.print("\t_config_to_send address: ");
            // Serial.println((long)&_config_to_send[0], HEX);
        #endif
        spi_data_idx::is_ff::clear_is_ff();
        for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
        {
            spi_data_idx::is_ff::is_ff[i] = _peripheral_message[i+spi_data_idx::base_idx_cnt];
            #ifdef SPI_DEBUG
                Serial.print("SPIHandler::_parse_config : is_ff[");
                Serial.print(i);
                Serial.print("] = 0b");
                Serial.println(spi_data_idx::is_ff::is_ff[i],BIN);
            #endif
        }
        

        for(int i = 0; i<ini_config::number_of_keys; i++)
        {
            #ifdef SPI_DEBUG
                
                Serial.print("\tunset _config_to_send = ");
                Serial.print(_config_to_send[i]);
            #endif
            _config_to_send[i] = ff_to_fe_check(_peripheral_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + i]);
            
            #ifdef SPI_DEBUG
                Serial.print("\t_peripheral_message = ");
                Serial.print(_peripheral_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + i]);
                Serial.print(" -> _config_to_send = ");
                Serial.println(_config_to_send[i]);
            #endif
        }
        
        _data_len = get_data_len(_config_to_send);
        _msg_len = _padding + max(max(spi_cmd::max_param_len, _data_len), ini_config::number_of_keys) + spi_data_idx::is_ff::num_bytes;
    };
    
    void SPIHandler::_parse_data()
    {
        #ifdef SPI_DEBUG
            Serial.println("SPIHandler::_parse_data : Entered");
        #endif
        spi_data_idx::is_ff::clear_is_ff();
        for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
        {
            spi_data_idx::is_ff::is_ff[i] = _peripheral_message[i+spi_data_idx::base_idx_cnt];
            #ifdef SPI_DEBUG
                Serial.print("\tSPIHandler::_parse_data : is_ff[");
                Serial.print(i);
                Serial.print("] = 0b");
                Serial.println(spi_data_idx::is_ff::is_ff[i],BIN);
            #endif
        }
        
        uint8_t running_idx_cnt = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes;
        #ifdef SPI_DEBUG
            Serial.print("\t running_idx_cnt : ");
            Serial.println(running_idx_cnt);
            Serial.print("\t _peripheral_message[spi_data_idx::exo::exo_status] : ");
            Serial.println(_peripheral_message[running_idx_cnt + spi_data_idx::exo::exo_status]);
            Serial.print("\t _peripheral_message[spi_data_idx::exo::sync_led_state] : ");
            Serial.println(_peripheral_message[running_idx_cnt + spi_data_idx::exo::sync_led_state]);
        #endif
        // send exo top level data
        _data->status = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::exo::exo_status]);
        _data->sync_led_state = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::exo::sync_led_state]);
        running_idx_cnt += spi_data_idx::exo::idx_cnt;
        #ifdef SPI_DEBUG
            Serial.print("\t Status : ");
            Serial.println(_data->status);
            Serial.print("\t Sync LED : ");
            Serial.println(_data->sync_led_state);
        #endif
        
        
        // left leg used    
        if (_config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::bilateral || _config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::left)
        {
            _data->left_leg.do_calibration_heel_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_heel_fsr]);
            _data->left_leg.do_calibration_toe_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_toe_fsr]);
            
            _data->left_leg.do_calibration_refinement_heel_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_heel_fsr]);
            _data->left_leg.do_calibration_refinement_toe_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_toe_fsr]);
            
            _data->left_leg.percent_gait = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::leg::percent_gait);
            _data->left_leg.heel_fsr = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::leg::heel_fsr);
            _data->left_leg.toe_fsr = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::leg::toe_fsr);
            running_idx_cnt += spi_data_idx::leg::idx_cnt;
            
            #ifdef SPI_DEBUG
                Serial.print("\tLeft :: FSR Calibration : ");
                Serial.print(_data->left_leg.do_calibration_heel_fsr);
                Serial.println(_data->left_leg.do_calibration_toe_fsr);
                Serial.print("\tLeft :: FSR Refinement : ");
                Serial.print(_data->left_leg.do_calibration_refinement_heel_fsr);
                Serial.println(_data->left_leg.do_calibration_refinement_toe_fsr);
                Serial.print("\tLeft :: Percent Gait : ");
                Serial.println(_data->left_leg.percent_gait);
                // Serial.print("\tLeft :: Percent Gait Bytes: ");
                // for (unsigned int i = 0; i < sizeof(SPI_DATA_TYPE); i++)
                // {
                    // Serial.print(_peripheral_message[running_idx_cnt + spi_data_idx::leg::percent_gait-spi_data_idx::leg::idx_cnt+i],HEX);
                    // Serial.print("\t");
                // }
                // Serial.print("\n");
                Serial.print("\tLeft :: Heel FSR : ");
                Serial.println(_data->left_leg.heel_fsr);
                Serial.print("\tLeft :: Toe FSR : ");
                Serial.println(_data->left_leg.toe_fsr);
            #endif
        
            
            // left hip used
            if (_config_to_send[config_defs::hip_idx] != (uint8_t)config_defs::motor::not_used)
            {
                _data->left_leg.hip.calibrate_torque_sensor = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor]);
                _data->left_leg.hip.torque_reading = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::joint::torque);
                running_idx_cnt += spi_data_idx::joint::idx_cnt;
                
                _data->left_leg.hip.motor.p = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p);
                _data->left_leg.hip.motor.v = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v);
                _data->left_leg.hip.motor.i = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::i);
                _data->left_leg.hip.motor.p_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des);
                _data->left_leg.hip.motor.v_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des);
                _data->left_leg.hip.motor.t_ff = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff);
                running_idx_cnt += spi_data_idx::motor::idx_cnt;
                
                #ifdef SPI_DEBUG
                    Serial.println("\tLeft :: Hip");
                    Serial.print("\t\tcalibrate_torque_sensor : ");
                    Serial.println(_data->left_leg.hip.calibrate_torque_sensor);
                    Serial.print("\t\ttorque_reading : ");
                    Serial.println(_data->left_leg.hip.torque_reading);
                    Serial.print("\t\tMotor :: p : ");
                    Serial.println(_data->left_leg.hip.motor.p);
                    Serial.print("\t\tMotor :: v : ");
                    Serial.println(_data->left_leg.hip.motor.v);
                    Serial.print("\t\tMotor :: i : ");
                    Serial.println(_data->left_leg.hip.motor.i);
                    Serial.print("\t\tMotor :: p_des : ");
                    Serial.println(_data->left_leg.hip.motor.p_des);
                    Serial.print("\t\tMotor :: v_des : ");
                    Serial.println(_data->left_leg.hip.motor.v_des);
                    Serial.print("\t\tMotor :: t_ff : ");
                    Serial.println(_data->left_leg.hip.motor.t_ff);
                #endif
            }
            
            // left knee used
            if (_config_to_send[config_defs::knee_idx] != (uint8_t)config_defs::motor::not_used)
            {
                _data->left_leg.knee.calibrate_torque_sensor = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor]);
                _data->left_leg.knee.torque_reading = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::joint::torque);
                running_idx_cnt += spi_data_idx::joint::idx_cnt;
                
                _data->left_leg.knee.motor.p = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p);
                _data->left_leg.knee.motor.v = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v);
                _data->left_leg.knee.motor.i = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::i);
                _data->left_leg.knee.motor.p_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des);
                _data->left_leg.knee.motor.v_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des);
                _data->left_leg.knee.motor.t_ff = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff);
                running_idx_cnt += spi_data_idx::motor::idx_cnt;
                
                #ifdef SPI_DEBUG
                    Serial.println("\tLeft :: Knee");
                    Serial.print("\t\tcalibrate_torque_sensor : ");
                    Serial.println(_data->left_leg.knee.calibrate_torque_sensor);
                    Serial.print("\t\ttorque_reading : ");
                    Serial.println(_data->left_leg.knee.torque_reading);
                    Serial.print("\t\tMotor :: p : ");
                    Serial.println(_data->left_leg.knee.motor.p);
                    Serial.print("\t\tMotor :: v : ");
                    Serial.println(_data->left_leg.knee.motor.v);
                    Serial.print("\t\tMotor :: i : ");
                    Serial.println(_data->left_leg.knee.motor.i);
                    Serial.print("\t\tMotor :: p_des : ");
                    Serial.println(_data->left_leg.knee.motor.p_des);
                    Serial.print("\t\tMotor :: v_des : ");
                    Serial.println(_data->left_leg.knee.motor.v_des);
                    Serial.print("\t\tMotor :: t_ff : ");
                    Serial.println(_data->left_leg.knee.motor.t_ff);
                #endif
            }
            
            // left ankle used
            if (_config_to_send[config_defs::ankle_idx] != (uint8_t)config_defs::motor::not_used)
            {
                _data->left_leg.ankle.calibrate_torque_sensor = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor]);
                _data->left_leg.ankle.torque_reading = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::joint::torque);
                running_idx_cnt += spi_data_idx::joint::idx_cnt;
                
                _data->left_leg.ankle.motor.p = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p);
                _data->left_leg.ankle.motor.v = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v);
                _data->left_leg.ankle.motor.i = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::i);
                _data->left_leg.ankle.motor.p_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des);
                _data->left_leg.ankle.motor.v_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des);
                _data->left_leg.ankle.motor.t_ff = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff);
                running_idx_cnt += spi_data_idx::motor::idx_cnt;
                
                #ifdef SPI_DEBUG
                    Serial.println("\tLeft :: Ankle");
                    Serial.print("\t\tcalibrate_torque_sensor : ");
                    Serial.println(_data->left_leg.ankle.calibrate_torque_sensor);
                    Serial.print("\t\ttorque_reading : ");
                    Serial.println(_data->left_leg.ankle.torque_reading);
                    Serial.print("\t\tMotor :: p : ");
                    Serial.println(_data->left_leg.ankle.motor.p);
                    Serial.print("\t\tMotor :: v : ");
                    Serial.println(_data->left_leg.ankle.motor.v);
                    Serial.print("\t\tMotor :: i : ");
                    Serial.println(_data->left_leg.ankle.motor.i);
                    Serial.print("\t\tMotor :: p_des : ");
                    Serial.println(_data->left_leg.ankle.motor.p_des);
                    Serial.print("\t\tMotor :: v_des : ");
                    Serial.println(_data->left_leg.ankle.motor.v_des);
                    Serial.print("\t\tMotor :: t_ff : ");
                    Serial.println(_data->left_leg.ankle.motor.t_ff);
                #endif
            }
            
        }
        
        // right leg used
        if (_config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::bilateral || _config_to_send[config_defs::exo_side_idx] == (uint8_t)config_defs::exo_side::right)
        {
            _data->right_leg.do_calibration_heel_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_heel_fsr]);
            _data->right_leg.do_calibration_toe_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_toe_fsr]);
            
            _data->right_leg.do_calibration_refinement_heel_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_heel_fsr]);
            _data->right_leg.do_calibration_refinement_toe_fsr = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::leg::do_calibration_refinement_toe_fsr]);
            
            _data->right_leg.percent_gait = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::leg::percent_gait);
            _data->right_leg.heel_fsr = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::leg::heel_fsr);
            _data->right_leg.toe_fsr = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::leg::toe_fsr);
            running_idx_cnt += spi_data_idx::leg::idx_cnt;
            #ifdef SPI_DEBUG
                Serial.print("\tRight :: FSR Calibration : ");
                Serial.print(_data->right_leg.do_calibration_heel_fsr);
                Serial.println(_data->right_leg.do_calibration_toe_fsr);
                Serial.print("\tRight :: FSR Refinement : ");
                Serial.print(_data->right_leg.do_calibration_refinement_heel_fsr);
                Serial.println(_data->right_leg.do_calibration_refinement_toe_fsr);
                Serial.print("\tRight :: Percent Gait : ");
                Serial.println(_data->right_leg.percent_gait);
                Serial.print("\tLeft :: Heel FSR : ");
                Serial.println(_data->right_leg.heel_fsr);
                Serial.print("\tLeft :: Toe FSR : ");
                Serial.println(_data->right_leg.toe_fsr);
            #endif
            
            // right hip used
            if (_config_to_send[config_defs::hip_idx] != (uint8_t)config_defs::motor::not_used)
            {
                _data->right_leg.hip.calibrate_torque_sensor = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor]);
                _data->right_leg.hip.torque_reading = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::joint::torque);
                running_idx_cnt += spi_data_idx::joint::idx_cnt;
                
                _data->right_leg.hip.motor.p = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p);
                _data->right_leg.hip.motor.v = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v);
                _data->right_leg.hip.motor.i = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::i);
                _data->right_leg.hip.motor.p_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des);
                _data->right_leg.hip.motor.v_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des);
                _data->right_leg.hip.motor.t_ff = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff);
                running_idx_cnt += spi_data_idx::motor::idx_cnt;
                
                #ifdef SPI_DEBUG
                    Serial.println("\tRight :: Hip");
                    Serial.print("\t\tcalibrate_torque_sensor : ");
                    Serial.println(_data->right_leg.hip.calibrate_torque_sensor);
                    Serial.print("\t\ttorque_reading : ");
                    Serial.println(_data->right_leg.hip.torque_reading);
                    Serial.print("\t\tMotor :: p : ");
                    Serial.println(_data->right_leg.hip.motor.p);
                    Serial.print("\t\tMotor :: v : ");
                    Serial.println(_data->right_leg.hip.motor.v);
                    Serial.print("\t\tMotor :: i : ");
                    Serial.println(_data->right_leg.hip.motor.i);
                    Serial.print("\t\tMotor :: p_des : ");
                    Serial.println(_data->right_leg.hip.motor.p_des);
                    Serial.print("\t\tMotor :: v_des : ");
                    Serial.println(_data->right_leg.hip.motor.v_des);
                    Serial.print("\t\tMotor :: t_ff : ");
                    Serial.println(_data->right_leg.hip.motor.t_ff);
                #endif
            }
            
            // right knee used
            if (_config_to_send[config_defs::knee_idx] != (uint8_t)config_defs::motor::not_used)
            {
                _data->right_leg.knee.calibrate_torque_sensor = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor]);
                _data->right_leg.knee.torque_reading = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::joint::torque);
                running_idx_cnt += spi_data_idx::joint::idx_cnt;
                
                _data->right_leg.knee.motor.p = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p);
                _data->right_leg.knee.motor.v = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v);
                _data->right_leg.knee.motor.i = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::i);
                _data->right_leg.knee.motor.p_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des);
                _data->right_leg.knee.motor.v_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des);
                _data->right_leg.knee.motor.t_ff = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff);
                running_idx_cnt += spi_data_idx::motor::idx_cnt;
                
                #ifdef SPI_DEBUG
                    Serial.println("\tRight :: Knee");
                    Serial.print("\t\tcalibrate_torque_sensor : ");
                    Serial.println(_data->right_leg.knee.calibrate_torque_sensor);
                    Serial.print("\t\ttorque_reading : ");
                    Serial.println(_data->right_leg.knee.torque_reading);
                    Serial.print("\t\tMotor :: p : ");
                    Serial.println(_data->right_leg.knee.motor.p);
                    Serial.print("\t\tMotor :: v : ");
                    Serial.println(_data->right_leg.knee.motor.v);
                    Serial.print("\t\tMotor :: i : ");
                    Serial.println(_data->right_leg.knee.motor.i);
                    Serial.print("\t\tMotor :: p_des : ");
                    Serial.println(_data->right_leg.knee.motor.p_des);
                    Serial.print("\t\tMotor :: v_des : ");
                    Serial.println(_data->right_leg.knee.motor.v_des);
                    Serial.print("\t\tMotor :: t_ff : ");
                    Serial.println(_data->right_leg.knee.motor.t_ff);
                #endif
            }
            
            // right ankle used
            if (_config_to_send[config_defs::ankle_idx] != (uint8_t)config_defs::motor::not_used)
            {
                _data->right_leg.ankle.calibrate_torque_sensor = fe_to_ff_check(_peripheral_message[running_idx_cnt + spi_data_idx::joint::calibrate_torque_sensor]);
                _data->right_leg.ankle.torque_reading = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::joint::torque);
                running_idx_cnt += spi_data_idx::joint::idx_cnt;
                
                _data->right_leg.ankle.motor.p = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p);
                _data->right_leg.ankle.motor.v = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v);
                _data->right_leg.ankle.motor.i = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::i);
                _data->right_leg.ankle.motor.p_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::p_des);
                _data->right_leg.ankle.motor.v_des = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::v_des);
                _data->right_leg.ankle.motor.t_ff = _unpack_float(_peripheral_message, running_idx_cnt + spi_data_idx::motor::t_ff);
                running_idx_cnt += spi_data_idx::motor::idx_cnt;
                
                #ifdef SPI_DEBUG
                    Serial.println("\tRight :: Ankle");
                    Serial.print("\t\tcalibrate_torque_sensor : ");
                    Serial.println(_data->right_leg.ankle.calibrate_torque_sensor);
                    Serial.print("\t\ttorque_reading : ");
                    Serial.println(_data->right_leg.ankle.torque_reading);
                    Serial.print("\t\tMotor :: p : ");
                    Serial.println(_data->right_leg.ankle.motor.p);
                    Serial.print("\t\tMotor :: v : ");
                    Serial.println(_data->right_leg.ankle.motor.v);
                    Serial.print("\t\tMotor :: i : ");
                    Serial.println(_data->right_leg.ankle.motor.i);
                    Serial.print("\t\tMotor :: p_des : ");
                    Serial.println(_data->right_leg.ankle.motor.p_des);
                    Serial.print("\t\tMotor :: v_des : ");
                    Serial.println(_data->right_leg.ankle.motor.v_des);
                    Serial.print("\t\tMotor :: t_ff : ");
                    Serial.println(_data->right_leg.ankle.motor.t_ff);
                #endif
            }
        }
    };
    
    void SPIHandler::_pack_message()
    {
        spi_data_idx::is_ff::clear_is_ff();
            
        _controller_message[spi_data_idx::length_idx] = _msg_len;
        _controller_message[spi_data_idx::send_cmd_idx] = _command;
        switch (_command)
        {
            case spi_cmd::send_data_exo::id:
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                // potential to optimize with fill
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::send_config::id:
               
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::null_cmd::id:
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::update_controller::id:
                
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::joint_id_idx] = ff_to_fe_check(_joint_id);
                
                switch (_joint_id)
                {
                    
                    // left
                    case (uint8_t)config_defs::joint_id::left_hip:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx] = ff_to_fe_check(_data->left_leg.hip.controller.controller);
                        break;
                    case (uint8_t)config_defs::joint_id::left_knee:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx] = ff_to_fe_check(_data->left_leg.knee.controller.controller);
                        break;
                    case (uint8_t)config_defs::joint_id::left_ankle:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx] = ff_to_fe_check(_data->left_leg.ankle.controller.controller);
                        break;
                    // right
                    case (uint8_t)config_defs::joint_id::right_hip:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx] = ff_to_fe_check(_data->right_leg.hip.controller.controller);
                        break;
                    case (uint8_t)config_defs::joint_id::right_knee:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx] = ff_to_fe_check(_data->right_leg.knee.controller.controller);
                        break;
                    case (uint8_t)config_defs::joint_id::right_ankle:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::controller_idx] = ff_to_fe_check(_data->right_leg.ankle.controller.controller);
                        break;
                }
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                // fill in the rest of the message with zeros.  If the _data_len is is shorter than the controller message i will equal param length and the loop will be skipped.
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller::param_len; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::update_controller_params::id:
            
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::joint_id_idx] = ff_to_fe_check(_joint_id);
                
                switch (_joint_id)
                {
                    // left
                    case (uint8_t)config_defs::joint_id::left_hip:
                        for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                        {
                            _pack_float(_controller_message, spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + message_idx, _data->left_leg.hip.controller.parameters[parameter_idx]);
                        }
                        break;
                    case (uint8_t)config_defs::joint_id::left_knee:
                        for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                        {
                            _pack_float(_controller_message, spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + message_idx, _data->left_leg.knee.controller.parameters[parameter_idx]);
                        }
                        break;
                    case (uint8_t)config_defs::joint_id::left_ankle:
                        for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                        {
                            _pack_float(_controller_message, spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + message_idx, _data->left_leg.ankle.controller.parameters[parameter_idx]);
                        }
                        break;
                    // right
                    case (uint8_t)config_defs::joint_id::right_hip:
                        for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                        {
                            _pack_float(_controller_message, spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + message_idx, _data->right_leg.hip.controller.parameters[parameter_idx]);
                        }
                        break;
                    case (uint8_t)config_defs::joint_id::right_knee:
                        for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                        {
                            _pack_float(_controller_message, spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + message_idx, _data->right_leg.knee.controller.parameters[parameter_idx]);
                        }
                        break;
                    case (uint8_t)config_defs::joint_id::right_ankle:
                        for (int message_idx = spi_cmd::update_controller_params::param_start_idx, parameter_idx = 0; message_idx <= spi_cmd::update_controller_params::param_stop_idx; message_idx += sizeof(SPI_DATA_TYPE), parameter_idx++)
                        {
                            _pack_float(_controller_message, spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + message_idx, _data->right_leg.ankle.controller.parameters[parameter_idx]);
                        }
                        break;
                }
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                // fill in the rest of the message with zeros.  If the _data_len is is shorter than the controller message i will equal param length and the loop will be skipped.
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::update_controller_params::param_len; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                
                break;
            case spi_cmd::calibrate_torque_sensor::id:
                
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::joint_id_idx] = ff_to_fe_check(_joint_id);
                
                
                switch (_joint_id)
                {
                    // left
                    case (uint8_t)config_defs::joint_id::left_hip:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx] = ff_to_fe_check(_data->left_leg.hip.calibrate_torque_sensor);
                        break;
                    case (uint8_t)config_defs::joint_id::left_knee:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx] = ff_to_fe_check(_data->left_leg.knee.calibrate_torque_sensor);
                        break;
                    case (uint8_t)config_defs::joint_id::left_ankle:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx] = ff_to_fe_check(_data->left_leg.ankle.calibrate_torque_sensor);
                        break;
                    // right
                    case (uint8_t)config_defs::joint_id::right_hip:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx] = ff_to_fe_check(_data->right_leg.hip.calibrate_torque_sensor);
                        break;
                    case (uint8_t)config_defs::joint_id::right_knee:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx] = ff_to_fe_check(_data->right_leg.knee.calibrate_torque_sensor);
                        break;
                    case (uint8_t)config_defs::joint_id::right_ankle:
                        _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::calibrate_torque_sensor_idx] = ff_to_fe_check(_data->right_leg.ankle.calibrate_torque_sensor);
                        break;
                }
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                // fill in the rest of the message with zeros.  If the _data_len is is shorter than the controller message i will equal param length and the loop will be skipped.
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_torque_sensor::param_len; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::calibrate_fsr::id:
            
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::left_heel_idx] = ff_to_fe_check(_data->left_leg.do_calibration_heel_fsr);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::left_toe_idx] = ff_to_fe_check(_data->left_leg.do_calibration_toe_fsr);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::right_heel_idx] = ff_to_fe_check(_data->right_leg.do_calibration_heel_fsr);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::right_toe_idx] = ff_to_fe_check(_data->right_leg.do_calibration_toe_fsr);
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::calibrate_fsr::param_len; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::refine_fsr::id:
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::refine_fsr::left_heel_idx] = ff_to_fe_check(_data->left_leg.do_calibration_refinement_heel_fsr);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::refine_fsr::left_toe_idx] = ff_to_fe_check(_data->left_leg.do_calibration_refinement_toe_fsr);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::refine_fsr::right_heel_idx] = ff_to_fe_check(_data->right_leg.do_calibration_refinement_heel_fsr);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::refine_fsr::right_toe_idx] = ff_to_fe_check(_data->right_leg.do_calibration_refinement_toe_fsr);
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::refine_fsr::param_len; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::motor_enable_disable::id:
                
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_hip_idx] = ff_to_fe_check(_data->left_leg.hip.motor.enabled);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_knee_idx] = ff_to_fe_check(_data->left_leg.knee.motor.enabled);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_ankle_idx] = ff_to_fe_check(_data->left_leg.ankle.motor.enabled);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_hip_idx] = ff_to_fe_check(_data->right_leg.hip.motor.enabled);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_knee_idx] = ff_to_fe_check(_data->right_leg.knee.motor.enabled);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_ankle_idx] = ff_to_fe_check(_data->right_leg.ankle.motor.enabled);
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::param_len; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            case spi_cmd::motor_zero::id:
                
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_hip_idx] = ff_to_fe_check(_data->left_leg.hip.motor.do_zero);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_knee_idx] = ff_to_fe_check(_data->left_leg.knee.motor.do_zero);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::left_ankle_idx] = ff_to_fe_check(_data->left_leg.ankle.motor.do_zero);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_hip_idx] = ff_to_fe_check(_data->right_leg.hip.motor.do_zero);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_knee_idx] = ff_to_fe_check(_data->right_leg.knee.motor.do_zero);
                _controller_message[spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_enable_disable::right_ankle_idx] = ff_to_fe_check(_data->right_leg.ankle.motor.do_zero);
                
                for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                {
                    _controller_message[i+spi_data_idx::base_idx_cnt] = spi_data_idx::is_ff::is_ff[i];
                }
                
                for (int i = spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + spi_cmd::motor_zero::param_len; i<_msg_len; i++)
                {
                    _controller_message[i] = 0;
                }
                break;
            // case spi_cmd::::id:
                
                // break;               
        }
    };
    
    void SPIHandler::_send_message()
    {
        // send parameters
        SPI.beginTransaction(SPISettings(5000000, MSBFIRST, coms_micro_pins::spi_mode));
        digitalWrite(coms_micro_pins::cs_pin, LOW);
        #ifdef SPI_DEBUG
            Serial.print("SPIHandler::_send_message : command at start : ");
            print_message_name(_command);
            Serial.println("");
            Serial.println("SPIHandler::_send_message :  _peripheral_message");
        #endif
        
        for( int i = 0, j = 0; j<_msg_len; i++)
        {
            _peripheral_message[j] = SPI.transfer(_controller_message[i]);// send the data converting FF to FE so it isn't seen as error
            //_peripheral_message[j] = SPI.transfer(utils::ff_to_fe(_controller_message[i]));// send the data converting FF to FE so it isn't seen as error
            while(0xFF == _peripheral_message[j])// if error recieved send it back so the other system knows
            {
                _peripheral_message[j] = SPI.transfer(0xFF);
            }
            // echo the received command back 
            if((spi_data_idx::send_cmd_idx==j))
            {
                _controller_message[spi_data_idx::recv_cmd_idx] = _peripheral_message[j];
            }
            
            // if ((spi_data_idx::base_idx_cnt + spi_data_idx::base_idx_cnt)==j)
            // {
                // spi_data_idx::is_ff::clear_is_ff();
                // for(unsigned int i = 0; i<(spi_data_idx::is_ff::num_bytes); i++)
                // {
                    // spi_data_idx::is_ff::is_ff[i] = controller_message[i+spi_data_idx::base_idx_cnt];
                // }
            // }
            // read in the config if it is being sent then when complete set the message length
            if((_command == spi_cmd::send_config::id) && (j == (spi_data_idx::base_idx_cnt + spi_data_idx::base_idx_cnt + spi_data_idx::is_ff::num_bytes + ini_config::number_of_keys)))
            {
                _parse_config();
                _data_len = get_data_len(_config_to_send);
                _msg_len = _padding + max(max(spi_cmd::max_param_len, _data_len), ini_config::number_of_keys);
            }
            
            if((0xFF!=_peripheral_message[j]&&0!=j)||(0==j && (0xFF!=_peripheral_message[j]&&0!=_peripheral_message[j])))//remove leading 0s and ignore FF as the other side sends this back if it was received
            //if((0 == j && 0 != _peripheral_message[j])|| 0!=j)// ignore leading 0s
            {
              j++;
            }
        } 
        digitalWrite(coms_micro_pins::cs_pin, HIGH);
        SPI.endTransaction();
        
        #ifdef SPI_DEBUG
            Serial.print("SPIHandler::_send_message : command at end: ");
            print_message_name(_command);
            Serial.println("");
            Serial.print("SPIHandler::_send_message : _command address - _peripheral_message address: ");
            Serial.println((long)(&_command - &_peripheral_message[0]));

        #endif
    };
    
    void SPIHandler::_clear_message()
    {
        for( int i = 0; i<sizeof(_controller_message); i++)
        {
            _controller_message[i] = 0;
            _peripheral_message[i] = 0;
        }
        
        
    };
    
    
    

#endif