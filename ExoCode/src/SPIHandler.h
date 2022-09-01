/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef SPIHandler_h
#define SPIHandler_h

#include "Board.h"
#include "ParseIni.h"
#include "ExoData.h"
#include "Utilities.h"

#include "Arduino.h"
#include <stdint.h>
#include <vector>

#define MAX_NUM_LEGS 2
#define MAX_NUM_JOINTS_PER_LEG 2 // current PCB can only do 2 motors per leg.
// if type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define SPI_DATA_TYPE short int
#define FIXED_POINT_FACTOR 100
// TODO: create array to store change messages so that the specific changes can be made 


// requires both systems use the same side for floats, if this is an issue for your system you can use binary32
namespace spi_data_idx // read data that changes each loop
{
    
    const uint8_t length_idx = 1;
    const uint8_t send_cmd_idx = 0; //moved command first so things can be packed on the peripheral with a fixed value between
    const uint8_t recv_cmd_idx = 2; //moved command first so things can be packed on the peripheral with a fixed value between
    const uint8_t base_idx_cnt = 3;
    
    namespace is_ff
    {
        const uint8_t num_bytes = 3; // allows for num_bytes * 7 0xFE's
        extern uint8_t is_ff[num_bytes];
        extern uint8_t current_byte;
        extern uint8_t current_bit;
        extern bool overflow;
        const uint8_t start_idx = 0;  //uint8
        const uint8_t stop_idx = start_idx + num_bytes - 1;  // -1 is there because things are zero indexed, sanity check start_idx = 0, and num_bytes = 1
        void clear_is_ff(); 
        bool overflowcheck();
    }
    
    // exo specific
    namespace exo
    {
        const uint8_t idx_cnt = 1*sizeof(uint16_t) + 1;//0; //used to increment so we don't have to keep fiddling with numbers
        const uint16_t exo_status = 0;//idx_cnt++; uint16_t
        const uint16_t sync_led_state = 1*sizeof(uint16_t); //idx_cnt++;
    }
    
    // leg specific
    namespace leg
    {
        const uint8_t idx_cnt = 4 + 3 * sizeof(SPI_DATA_TYPE); //0; //used to increment so we don't have to keep fiddling with numbers
        const uint8_t do_calibration_heel_fsr = 0;
        const uint8_t do_calibration_toe_fsr = 1;
        const uint8_t do_calibration_refinement_heel_fsr = 2;
        const uint8_t do_calibration_refinement_toe_fsr = 3;
        const uint16_t percent_gait = 4; //idx_cnt++;  // float
        const uint16_t heel_fsr = percent_gait + 1 * sizeof(SPI_DATA_TYPE); //idx_cnt++;  // float
        const uint16_t toe_fsr = percent_gait + 2 * sizeof(SPI_DATA_TYPE); //idx_cnt++;  // float
    }
    
    // joint specific
    namespace joint
    {
        const uint8_t idx_cnt = 1 + 1 * sizeof(SPI_DATA_TYPE); //0; //used to increment so we don't have to keep fiddling with numbers
        const uint16_t calibrate_torque_sensor = 0; //idx_cnt++;
        const uint16_t torque = 1; //idx_cnt++; // float
    }
    
    // motor specific, includes controller cmd that was sent to the motor.
    namespace motor
    {
        const uint8_t idx_cnt = 6 * sizeof(SPI_DATA_TYPE); //0; //used to increment so we don't have to keep fiddling with numbers
        const uint16_t p = 0; //idx_cnt++;
        const uint16_t v = 1 * sizeof(SPI_DATA_TYPE); //idx_cnt++;
        const uint16_t i = 2 * sizeof(SPI_DATA_TYPE); //idx_cnt++;
        const uint16_t p_des = 3 * sizeof(SPI_DATA_TYPE); //idx_cnt++;
        const uint16_t v_des = 4 * sizeof(SPI_DATA_TYPE); //idx_cnt++;
        const uint16_t t_ff = 5 * sizeof(SPI_DATA_TYPE); //idx_cnt++;
    }
    
    
}

namespace spi_cmd
{
    // This can potentially cause issues in the specific case that these values appear in order since they are not reserved values.  This can be made less common by increasing the length, but will increase transmission time.
    const uint8_t start_stop_len = 2;
    const std::vector <uint8_t> start_message {0x57, 0xA7}; // tried spelling start in two bytes
    const std::vector <uint8_t> stop_message{0x57, 0xD9};  // tried spelling stop in two bytes
        
    
    
    const uint8_t max_data_len = spi_data_idx::exo::idx_cnt + MAX_NUM_LEGS * (spi_data_idx::leg::idx_cnt + MAX_NUM_JOINTS_PER_LEG * (spi_data_idx::joint::idx_cnt + spi_data_idx::motor::idx_cnt));
    
    namespace null_cmd
    {
        const uint8_t id = 0xFE;
        const uint8_t param_len = 0;  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        
    }
    // Request data 
    namespace send_config
    {
        const uint8_t id = 0x01;
        const uint8_t param_len = ini_config::number_of_keys;  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        // const uint16_t sample_message[] = {0x1111, 0x2222};
    }
    // Request data 
    namespace send_data_exo
    {
        const uint8_t id = send_config::id+1;
        const uint8_t param_len = max_data_len;  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        // const uint16_t sample_message[] = {0x1111, 0x2222};
    }
    // namespace send_data_leg
    // {
        // const uint8_t id = send_data_exo::id+1;
        // const uint8_t msg_len = spi_data_idx::leg::idx_cnt;  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        // // const uint16_t sample_message[] = {0x4444, 0x5555, 0x6666};
    // }
    // namespace send_data_joint
    // {
        // const uint8_t id = send_data_leg::id+1;
        // const uint8_t msg_len = spi_data_idx::joint::idx_cnt;  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        // // const uint16_t sample_message[] = {0x7777};
    // }
    // namespace send_data_motor
    // {
        // const uint8_t id = ::id+1;
        // const uint8_t msg_len = spi_data_idx::motor::idx_cnt;  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        // // const uint16_t sample_message[] = {0x8888, 0x9999, 0xAAAA, 0xBBBB, 0xCCCC, 0x7777};
    // }
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Update data second half of data
    namespace update_controller
    {
        const uint8_t id = send_config::id+2;
        const uint8_t joint_id_idx = 0;
        const uint8_t controller_idx = 1;
        const uint8_t param_len = 2;  // joint id, controller
    }
    namespace update_controller_params
    {
        const uint8_t id = send_config::id+3;
        const uint8_t joint_id_idx = 0; //uint8
        const uint8_t controller_idx = 1;
        const uint8_t param_start_idx = 2;  //uint8
        const uint8_t param_stop_idx = param_start_idx + sizeof(SPI_DATA_TYPE) * controller_defs::max_parameters - 1;  // -1 is there because things are zero indexed, sanity check start_idx = 0, sizeof(float) is 4, and max_parameters = 1
        
        const uint8_t param_len = 2 + param_stop_idx;  // joint id, controller
    }
    
    namespace calibrate_torque_sensor
    {
        const uint8_t id = send_config::id+4;
        const uint8_t joint_id_idx = 0;
        const uint8_t calibrate_torque_sensor_idx = 1;
        
        const uint8_t param_len = 2;  // joint id, do_calibrate
    }
    
    namespace calibrate_fsr
    {
        const uint8_t id = send_config::id+5;
        const uint8_t left_heel_idx = 0;
        const uint8_t left_toe_idx = 1;
        const uint8_t right_heel_idx = 2;
        const uint8_t right_toe_idx = 3;
        
        const uint8_t param_len = 4;  // individual sensor array
    }
    
    namespace refine_fsr
    {
        const uint8_t id = send_config::id+6;
        const uint8_t left_heel_idx = 0;
        const uint8_t left_toe_idx = 1;
        const uint8_t right_heel_idx = 2;
        const uint8_t right_toe_idx = 3;
        
        const uint8_t param_len = 4;  // individual sensor array
    }
    
    namespace motor_enable_disable
    {
        const uint8_t id = send_config::id+7;
        const uint8_t left_hip_idx = 0;
        const uint8_t left_knee_idx = 1;
        const uint8_t left_ankle_idx = 2;
        const uint8_t right_hip_idx = 3;
        const uint8_t right_knee_idx = 4;
        const uint8_t right_ankle_idx = 5;
        
        const uint8_t param_len = 6;  // individual sensor array
    }
    namespace motor_zero
    {
        const uint8_t id = send_config::id+8;
        const uint8_t left_hip_idx = 0;
        const uint8_t left_knee_idx = 1;
        const uint8_t left_ankle_idx = 2;
        const uint8_t right_hip_idx = 3;
        const uint8_t right_knee_idx = 4;
        const uint8_t right_ankle_idx = 5;
        
        const uint8_t param_len = 6;  // individual sensor array
    }
    // Update status value
    namespace update_status
    {
        const uint8_t id = send_config::id+9;
        const uint8_t status_idx = 0;
        const uint8_t param_len = sizeof(uint16_t);
    }
    
    namespace update_controller_params_workaround
    {
        const uint8_t id = send_config::id+10;
        const uint8_t joint_id_idx = 0; //uint8
        const uint8_t controller_idx = 1;
        const uint8_t parameter_set_idx = 2;  //uint8
        const uint8_t param_len = 3;  // joint id, controller
    }
    
    namespace calibrate_fsr_workaround
    {
        const uint8_t id = send_config::id+11;
        const uint8_t left_heel_cal_idx = 0;
        const uint8_t left_toe_cal_idx = 1;
        const uint8_t right_heel_cal_idx = 2;
        const uint8_t right_toe_cal_idx = 3;
        const uint8_t left_heel_refine_idx = 4;
        const uint8_t left_toe_refine_idx = 5;
        const uint8_t right_heel_refine_idx = 6;
        const uint8_t right_toe_refine_idx = 7;
        const uint8_t param_len = 8;  // joint id, controller
    }
    
    // check that this is the largest message len that will come in.
    const uint8_t max_param_len = update_controller_params::param_len;
    
    // Biofeedback
    
    
    // namespace update_motor_params
    // {
        // const uint8_t id = send_config::id+3;
        // const uint8_t msg_len = 0;
    // }
    

    // =============================
    namespace msg_template
    {
        const uint8_t id = 0x00;
        const uint8_t msg_len = 0;
    }
    //==============================
    
    
    
}
/*
 * Takes in the in a byte and if it is 0xFF changes to 0xFE
 * Takes in a byte array that records if the FE's are really FF(1) or FE(0)
 * Takes in the current bit for is_ff and increments if we have written is_ff
 * Takes in the current byte for is_ff and increments if we have written is_ff
 *      if current bit reaches 7 rolls over to 0 since we can only go as high as F0xFE
 */
void ff_to_fe(uint8_t* val, uint8_t* is_ff, uint8_t* current_byte, uint8_t* current_bit);

/*
 * Takes in the in a byte and if it is 0xFE and is supposed to be 0xFF changes to 0xFF
 * Takes in a byte that records if the FE's are really FF(1) or FE(0)
 * Takes in the current bit for is_ff and increments if the value was 0xFE
 */
void fe_to_ff(uint8_t* val, uint8_t* is_ff, uint8_t* current_byte, uint8_t* current_bit);

uint8_t ff_to_fe_check(uint8_t val);
uint8_t fe_to_ff_check(uint8_t val);

void print_message_name(uint8_t msg_id);
void print_config(uint8_t* config_to_send);
void print_message(uint8_t* message, uint8_t len);

uint8_t get_data_len(uint8_t* config_to_send); // done
        
#if defined(ARDUINO_TEENSY36)
    #include "TSPISlave.h"
    
    void spi_msg(TSPISlave, uint16_t *buffer );

//=======================================

#elif defined(ARDUINO_TEENSY41)
    #include "SPISlave_T4.h"
    
    namespace static_spi_handler
    {
        uint8_t peripheral_transaction(SPISlave_T4<&SPI, SPI_8_BITS> my_spi, uint8_t* config_to_send, ExoData* data); // done
        const uint8_t padding = spi_data_idx::base_idx_cnt + 1;//one extra for length, one for cmd, one for the end padding
        void pack_float(uint8_t* message, uint8_t start_idx, float val);  // done
        float unpack_float(uint8_t* message, uint8_t start_idx);  // done
        
        
        void pack_config(uint8_t* peripheral_message, uint8_t* config_to_send, uint8_t msg_len); // done
        void pack_data(uint8_t* peripheral_message, uint8_t cmd, uint8_t* config_to_send, ExoData* data, uint8_t msg_len); // done
        void parse_message(uint8_t* controller_message, uint8_t command, ExoData* data);  // done
        uint8_t read_command(uint8_t* controller_message);  // done
        void read_message(SPISlave_T4<&SPI, SPI_8_BITS> my_spi, uint8_t* controller_message); // done
        void clear_message(uint8_t* controller_message); // done
        void print_message(uint8_t* controller_message, ExoData* data, uint8_t command); // done
        void set_message_flag(bool* is_unread_message);
        void print_debug(uint16_t debug_location);
        
        void unpack_null_cmd(uint8_t* controller_message, ExoData* data);
        void unpack_send_config(uint8_t* controller_message, ExoData* data);
        void unpack_send_data_exo(uint8_t* controller_message, ExoData* data);
        void unpack_update_controller(uint8_t* controller_message, ExoData* data);
        void unpack_update_controller_params(uint8_t* controller_message, ExoData* data);
        void unpack_calibrate_torque_sensor(uint8_t* controller_message, ExoData* data);
        void unpack_calibrate_fsr(uint8_t* controller_message, ExoData* data);
        void unpack_refine_fsr(uint8_t* controller_message, ExoData* data);
        void unpack_motor_enable_disable(uint8_t* controller_message, ExoData* data);
        void unpack_motor_zero(uint8_t* controller_message, ExoData* data);
        void unpack_update_status(uint8_t* controller_message, ExoData* data);
        void unpack_update_controller_params_workaround(uint8_t* controller_message, ExoData* data);
        void unpack_calibrate_fsr_workaround(uint8_t* controller_message, ExoData* data);
        
        
        
        // debug locations
        const uint8_t debug_entry_bit = 0;
        const uint8_t debug_sent_length_bit = 1;
        const uint8_t debug_send_config_bit = 2;
        const uint8_t debug_send_data_bit = 3;
        const uint8_t debug_q_bit = 4;
        const uint8_t debug_w_bit = 5;
        const uint8_t debug_e_bit = 6;
        const uint8_t debug_r_bit = 7;
        const uint8_t debug_t_bit = 8;
        const uint8_t debug_y_bit = 9;
        const uint8_t debug_u_bit = 10;
        const uint8_t debug_i_bit = 11;
        const uint8_t debug_o_bit = 12;
        const uint8_t debug_p_bit = 13;
        const uint8_t debug_a_bit = 14;
        const uint8_t debug_exit_bit = 15;
        
    };
    
    // void spi_msg(SPISlave_T4<&SPI, SPI_8_BITS>, uint8_t buffer_len, uint8_t *config, uint8_t &buffer );

#elif defined(ARDUINO_ARDUINO_NANO33BLE)
    #include <SPI.h>
    
    /*
     * Need to have SPI enabled in ino
     */
    class SPIHandler
    {
        public:
            SPIHandler(uint8_t* config_to_send, ExoData* exo_data);
            
            void transaction(uint8_t command); // done
            void transaction(uint8_t command, uint8_t joint_id); // done
            
        private:
            //SPISlave_T4<&SPI, SPI_8_BITS> _my_spi;
            uint8_t _data_len;
            static const uint8_t _padding = spi_data_idx::base_idx_cnt + 1;//one extra for length, one for cmd, one for the end padding
            uint8_t _msg_len;
            uint8_t _joint_id;
            uint8_t _controller_message[_padding+spi_cmd::max_data_len+spi_data_idx::is_ff::num_bytes];  // Should be largest of parameters, data, and config length.
            uint8_t _peripheral_message[_padding+spi_cmd::max_data_len+spi_data_idx::is_ff::num_bytes]; //, ini_config::number_of_keys)]; // use which is bigger, this will almost always be the data length.
            uint8_t _command;
            uint8_t* _config_to_send;  // holds and array.
            
            
            
            static uint8_t _get_data_len(uint8_t *config_to_send); // done
            static void _pack_float(uint8_t* message, uint8_t start_idx, float val);  // done
            static float _unpack_float(uint8_t* message, uint8_t start_idx);  // done
            
            void _parse_message(); //done
            void _parse_config(); // done
            void _parse_data();  // done
            void _pack_message(); // done
            void _send_message(); // done
            void _clear_message(); // done
            
            void _pack_null_cmd(); 
            void _pack_send_config(); 
            void _pack_send_data_exo(); 
            void _pack_update_controller(); 
            void _pack_update_controller_params(); 
            void _pack_calibrate_torque_sensor(); 
            void _pack_calibrate_fsr(); 
            void _pack_refine_fsr(); 
            void _pack_motor_enable_disable(); 
            void _pack_motor_zero(); 
            void _pack_update_status(); 
            void _pack_update_controller_params_workaround(); 
            void _pack_calibrate_fsr_workaround(); 
            
            ExoData* _data;  // pointer to ExoData that is getting updated by SPI so they share memory.
        
    };
    
    // void spi_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, uint8_t *buffer);

#endif

#endif


