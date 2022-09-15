/**
 * @file SPIHandler.h
 *
 * @brief Declares a class used to transmit data between boards
 * 
 * @author P. Stegall 
 * @date Jan. 2022
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

#define MAX_NUM_LEGS 2 // max number of legs used to determine size of the data to be transmitted.
#define MAX_NUM_JOINTS_PER_LEG 2 // current PCB can only do 2 motors per leg used to determine size of the data to be transmitted.
// if type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define SPI_DATA_TYPE short int
#define FIXED_POINT_FACTOR 100 // precision for fixed point.  This will be multiplied by floats before changed to an int.
// TODO: create array to store change messages so that the specific changes can be made 


// requires both systems use the same side for floats, if this is an issue for your system you can use binary32
/**
 * @brief stores the index values of different data used to pack and unpack data.
 */
namespace spi_data_idx // read data that changes each loop
{
    
    const uint8_t length_idx = 1; /**< Byte in message which contains the length of the message*/
    const uint8_t send_cmd_idx = 0; /**< Command location to pack the controllers message, on the receiving side this will be where the external message is.  Moved command first so things can be packed on the peripheral with a fixed value between */
    const uint8_t recv_cmd_idx = 2; /**< Location of to echo the message coming from the external system.  Can be used to check that the external system correctly received the command*/
    const uint8_t base_idx_cnt = 3; /**< Number of bytes for the message meta data */
    
    /**
     * @brief bytes used to store if 0xFE received is actually an 0xFF. 
     * This is due to an error with the library where the teensy would miss a byte and the line would stay high resulting in the Nano receiving an FF. 
     * So when we want to send FF we convert it to FE and mark the corresponding bit here to 1.  
     * When we send FE we set the corresponding bit here to 0.
     */
    namespace is_ff
    {
        const uint8_t num_bytes = 3; /**< Number of bytes to allocate for storing info about 0xFF and 0xFE. Allows for num_bytes * 7 0xFE's */
        extern uint8_t is_ff[num_bytes]; /**< stores the bytes about 0xFF and 0xFE.*/
        extern uint8_t current_byte; /**< Current byte in is_ff being written/read*/
        extern uint8_t current_bit; /**< Current bit in the current byte in is_ff being written/read*/
        extern bool overflow; /**< Has the allocated space been exceeded*/
        const uint8_t start_idx = 0;  /**< The relative position where the bytes start*/
        const uint8_t stop_idx = start_idx + num_bytes - 1;  /**< where the is_ff values stop. -1 is there because things are zero indexed, sanity check start_idx = 0, and num_bytes = 1 */
        
        /**
         * @brief clears all of the bytes in is_ff
         */
        void clear_is_ff(); 
        
        /**
         * @brief returns a 1 if the current bit in the current byte exceeds the available space
         *
         * @return  1 if the current bit in the current byte exceeds the available space 0 otherwise
         */
        bool overflowcheck();
    }
    
    /**
     * @brief stores the index values to pack/unpack the exo specific values.
     */
    namespace exo
    {
        const uint8_t idx_cnt = 1*sizeof(uint16_t) + 1;//0; /**< Stores the number of bytes to allocate for the exo bytes.  Used to increment so we don't have to keep fiddling with numbers*/
        const uint16_t exo_status = 0;/**< Relative location of the exo status, uint16_t */
        const uint16_t sync_led_state = 1*sizeof(uint16_t); /**< Relative location of the sync led state */
    }
    
    /**
     * @brief stores the index values to pack/unpack the leg specific values.
     */
    namespace leg
    {
        const uint8_t idx_cnt = 4 + 3 * sizeof(SPI_DATA_TYPE); /**< Stores the number of bytes to allocate for the leg bytes.  Used to increment so we don't have to keep fiddling with numbers*/
        const uint8_t do_calibration_heel_fsr = 0; /**< Relative location of do_calibration_heel_fsr */
        const uint8_t do_calibration_toe_fsr = 1; /**< Relative location of do_calibration_toe_fsr */
        const uint8_t do_calibration_refinement_heel_fsr = 2; /**< Relative location of do_calibration_refinement_heel_fsr */
        const uint8_t do_calibration_refinement_toe_fsr = 3; /**< Relative location of do_calibration_refinement_toe_fsr */
        const uint16_t percent_gait = 4; /**< Relative location of the percent gait, float */
        const uint16_t heel_fsr = percent_gait + 1 * sizeof(SPI_DATA_TYPE); /**< Relative location of the heel_fsr, float */
        const uint16_t toe_fsr = percent_gait + 2 * sizeof(SPI_DATA_TYPE); /**< Relative location of the toe_fsr, float */  
    }
    
    /**
     * @brief stores the index values to pack/unpack the joint specific values.
     */
    namespace joint
    {
        const uint8_t idx_cnt = 1 + 1 * sizeof(SPI_DATA_TYPE); /**< Stores the number of bytes to allocate for the joint bytes.  Used to increment so we don't have to keep fiddling with numbers*/
        const uint16_t calibrate_torque_sensor = 0; /**< Relative location of calibrate_torque_sensor  */
        const uint16_t torque = 1; /**< Relative location of the torque, float */
    }
    
    /**
     * @brief stores the index values to pack/unpack the exo specific values.
     * Motor specific, includes controller cmd that was sent to the motor.
     */ 
    namespace motor
    {
        const uint8_t idx_cnt = 6 * sizeof(SPI_DATA_TYPE); /**< Stores the number of bytes to allocate for the motor bytes.  Used to increment so we don't have to keep fiddling with numbers*/
        const uint16_t p = 0; /**< Relative location of the position */
        const uint16_t v = 1 * sizeof(SPI_DATA_TYPE); /**< Relative location of the velocity, float */
        const uint16_t i = 2 * sizeof(SPI_DATA_TYPE); /**< Relative location of the current, float */
        const uint16_t p_des = 3 * sizeof(SPI_DATA_TYPE); /**< Relative location of the desired position, float */
        const uint16_t v_des = 4 * sizeof(SPI_DATA_TYPE); /**< Relative location of the desired velocity, float */
        const uint16_t t_ff = 5 * sizeof(SPI_DATA_TYPE); /**< Relative location of the torque command, float */
    }
    
    
}


/**
 * @brief stores the list of commands about what data is being sent between boards
 */
namespace spi_cmd
{
    // This can potentially cause issues in the specific case that these values appear in order since they are not reserved values.  This can be made less common by increasing the length, but will increase transmission time.
    const uint8_t start_stop_len = 2; /**< The number of byte is the start and stop messages */
    const std::vector <uint8_t> start_message {0x57, 0xA7}; /**< Bytes to appear at the start of a message, tried spelling start in two bytes */
    const std::vector <uint8_t> stop_message{0x57, 0xD9};  /**< Bytes to appear at the end of a message, tried spelling stop in two bytes */ 
        
    
    
    const uint8_t max_data_len = spi_data_idx::exo::idx_cnt + MAX_NUM_LEGS * (spi_data_idx::leg::idx_cnt + MAX_NUM_JOINTS_PER_LEG * (spi_data_idx::joint::idx_cnt + spi_data_idx::motor::idx_cnt)); /**< Maximum possible length of a message, used to allocate space */
    
    /**
     * @brief null message
     */
    namespace null_cmd
    {
        const uint8_t id = 0xFE; /**< id of message*/
        const uint8_t param_len = 0;  /**< size of the data being sent in the message*/ // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        
    }
    
    /**
     * @brief test command with some data
     */
    namespace test_cmd
    {
        const uint8_t id = 0xFD; /**< id of message*/
        const uint8_t val_idx = 0; /**< location of the test data in the message */
        const uint8_t param_len = 1*sizeof(SPI_DATA_TYPE); /**< size of the data being sent in the message*/  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        
    }
    
    /**
     * @brief Request config
     */
    namespace send_config
    {
        const uint8_t id = 0x01; /**< id of message*/
        const uint8_t param_len = ini_config::number_of_keys; /**< size of the data being sent in the message*/  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
        // const uint16_t sample_message[] = {0x1111, 0x2222};
    }
    /**
     * @brief Request data 
     */
    namespace send_data_exo
    {
        const uint8_t id = send_config::id+1; /**< id of message*/
        const uint8_t param_len = max_data_len; /**< size of the data being sent in the message*/  // need to decide if we want to separate by class type, e.g. read exo, read leg, read motor and just pull in the specific info, this could speed up unilateral system or systems where joints are not used.
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
    /**
     * @brief update the controller being used, parameters should be changed first from a safe controller
     */
    namespace update_controller
    {
        const uint8_t id = send_config::id+2; /**< id of message*/
        const uint8_t joint_id_idx = 0; /**< location of the joint id in the message */
        const uint8_t controller_idx = 1; /**< location of the controller number in the message */
        const uint8_t param_len = 2; /**< size of the data being sent in the message*/  // joint id, controller
    }
    
    /**
     * @brief update the controller parameters.  The controller type should be safe before doing this.
     */
    namespace update_controller_params
    {
        const uint8_t id = send_config::id+3; /**< id of message*/
        const uint8_t joint_id_idx = 0;  /**< location of the joint id in the message */
        const uint8_t controller_idx = 1;  /**< location of the controller id in the message */
        const uint8_t param_start_idx = 2;  /**< location of the start of the parameters in the message */ //uint8
        const uint8_t param_stop_idx = param_start_idx + sizeof(SPI_DATA_TYPE) * controller_defs::max_parameters - 1;   /**< location of the end of the parameters in the message */// -1 is there because things are zero indexed, sanity check start_idx = 0, sizeof(float) is 4, and max_parameters = 1
        
        const uint8_t param_len = 2 + param_stop_idx; /**< size of the data being sent in the message*/  // joint id, controller
    }
    
    /**
     * @brief sends what torque sensors should be calibrated
     */
    namespace calibrate_torque_sensor
    {
        const uint8_t id = send_config::id+4; /**< id of message*/
        const uint8_t joint_id_idx = 0; /**< location of the joint id in the message */
        const uint8_t calibrate_torque_sensor_idx = 1;  /**< location of the calibrate torque sensor values in the message */
        
        const uint8_t param_len = 2; /**< size of the data being sent in the message*/  // joint id, do_calibrate
    }
    
    /**
     * @brief Sends what FSRs should be calibrated
     */
    namespace calibrate_fsr
    {
        const uint8_t id = send_config::id+5; /**< id of message*/
        const uint8_t left_heel_idx = 0; /**< location of the left heel calibrate value in the message */
        const uint8_t left_toe_idx = 1; /**< location of the left toe calibrate value in the message */
        const uint8_t right_heel_idx = 2; /**< location of the right heel calibrate value in the message */
        const uint8_t right_toe_idx = 3; /**< location of the right toe calibrate value in the message */
        
        const uint8_t param_len = 4; /**< size of the data being sent in the message*/  // individual sensor array
    }
    
    /**
     * @brief Sends what FSRs should be refined
     */
    namespace refine_fsr
    {
        const uint8_t id = send_config::id+6; /**< id of message*/
        const uint8_t left_heel_idx = 0; /**< location of the left heel refine value in the message */
        const uint8_t left_toe_idx = 1; /**< location of the left toe refine value in the message */
        const uint8_t right_heel_idx = 2; /**< location of the right heel refine value in the message */
        const uint8_t right_toe_idx = 3; /**< location of the right toe refine value in the message */
        
        const uint8_t param_len = 4; /**< size of the data being sent in the message*/  // individual sensor array
    }
    
    /**
     * @brief sends the state to set different motors to.
     */
    namespace motor_enable_disable
    {
        const uint8_t id = send_config::id+7; /**< id of message*/
        const uint8_t left_hip_idx = 0; /**< location of the left hip enable in the message */
        const uint8_t left_knee_idx = 1; /**< location of the left knee enable in the message */
        const uint8_t left_ankle_idx = 2; /**< location of the left ankle enable in the message */
        const uint8_t right_hip_idx = 3; /**< location of the right hip enable in the message */
        const uint8_t right_knee_idx = 4; /**< location of the right knee enable in the message */
        const uint8_t right_ankle_idx = 5; /**< location of the right ankle enable in the message */
        
        const uint8_t param_len = 6; /**< size of the data being sent in the message*/  // individual sensor array
    }
    
    /**
     * @brief sends what motors should have their position zeroed
     */
    namespace motor_zero
    {
        const uint8_t id = send_config::id+8; /**< id of message*/
        const uint8_t left_hip_idx = 0; /**< location of the left hip do_zero in the message */
        const uint8_t left_knee_idx = 1; /**< location of the left knee do_zero in the message */
        const uint8_t left_ankle_idx = 2; /**< location of the left ankle do_zero in the message */
        const uint8_t right_hip_idx = 3; /**< location of the right hip do_zero in the message */
        const uint8_t right_knee_idx = 4; /**< location of the right knee do_zero in the message */
        const uint8_t right_ankle_idx = 5; /**< location of the right ankle do_zero in the message */
        
        const uint8_t param_len = 6; /**< size of the data being sent in the message*/  // individual sensor array
    }
    
    
    /**
     * @brief Update status value, such as starting/ending a trial
     */
    namespace update_status
    {
        const uint8_t id = send_config::id+9; /**< id of message*/
        const uint8_t status_idx = 0; /**< location of the  in the message */
        const uint8_t param_len = sizeof(uint16_t); /**< size of the data being sent in the message*/
    }
    
    /**
     * @brief A workaround allowing for the parameters to be set using existing app fields pulling the parameter set from the SD card.
     */
    namespace update_controller_params_workaround
    {
        const uint8_t id = send_config::id+10; /**< id of message*/
        const uint8_t joint_id_idx = 0; /**< location of the  in the message */ //uint8
        const uint8_t controller_idx = 1; /**< location of the  in the message */
        const uint8_t parameter_set_idx = 2; /**< location of the  in the message */  //uint8
        const uint8_t param_len = 3; /**< size of the data being sent in the message*/  // joint id, controller
    }
    
    /**
     * @brief A workaround allowing for the fsrs to be calibrated and refined together using existing fields in the app.
     */
    namespace calibrate_fsr_workaround
    {
        const uint8_t id = send_config::id+11; /**< id of message*/
        const uint8_t left_heel_cal_idx = 0; /**< location of the  in the message */
        const uint8_t left_toe_cal_idx = 1;
        const uint8_t right_heel_cal_idx = 2;
        const uint8_t right_toe_cal_idx = 3;
        const uint8_t left_heel_refine_idx = 4;
        const uint8_t left_toe_refine_idx = 5;
        const uint8_t right_heel_refine_idx = 6;
        const uint8_t right_toe_refine_idx = 7;
        const uint8_t param_len = 8; /**< size of the data being sent in the message*/  // joint id, controller
    }
    
    // check that this is the largest message len that will come in.
    const uint8_t max_param_len = update_controller_params::param_len; /**< Size of the largest possible message, this should be the max of all param_lens */
    
    // Biofeedback
    
    
    // namespace update_motor_params
    // {
        // const uint8_t id = send_config::id+3;
        // const uint8_t msg_len = 0;
    // }
    

    // =============================
    /**
     * @brief message template that can be used as the base to create new ones
     */
    namespace msg_template
    {
        const uint8_t id = 0x00; /**< id of message*/
        const uint8_t param_len = 0;  /**< size of the data being sent in the message*/
    }
    //==============================
    
    
    
}

/**
 * @brief Tracks if a byte is 0xFF if it is changes it to FE and records that it was changed, if a value is FE records that it was not changed, all other values have no action taken.
 * 
 * @param A byte and if it is 0xFF changes to 0xFE
 * @param A byte array that records if the FE's are really FF(1) or FE(0)
 * @param The current byte for is_ff and increments if we have written is_ff
 * @param The current bit for is_ff and increments if we have written is_ff
 *      if current bit reaches 7 rolls over to 0 since we can only go as high as 0xFE and move to the next byte
 */
void ff_to_fe(uint8_t* val, uint8_t* is_ff, uint8_t* current_byte, uint8_t* current_bit);

/**
 * @brief Decodes if an 0xFE byte is actually 0xFE or needs to be changed to FF.
 * 
 * @param A byte to check and if it is 0xFE and is supposed to be 0xFF changes to 0xFF
 * @param A byte array that records if the FE's are really FF(1) or FE(0)
 * @param The current byte that is being used to decode information
 * @param Takes in the current bit for is_ff and increments if the value was 0xFE
 */
void fe_to_ff(uint8_t* val, uint8_t* is_ff, uint8_t* current_byte, uint8_t* current_bit);

/**
 * @brief Tracks if a byte is 0xFF if it is changes it to FE and records that it was changed, if a value is FE records that it was not changed, all other values have no action taken.
 * 
 * @param A byte and if it is 0xFF changes to 0xFE
 * 
 * @return the input byte appropriately modified or unmodified
 */
uint8_t ff_to_fe_check(uint8_t val);

/**
 * @brief Decodes if an 0xFE byte is actually 0xFE or needs to be changed to FF.
 * 
 * @param A byte to check and if it is 0xFE and is supposed to be 0xFF changes to 0xFF
 * 
 * @return the input byte appropriately modified or unmodified
 */
uint8_t fe_to_ff_check(uint8_t val);

/**
 * @brief prints the name of the message that was received
 *
 * @param message id
 */
void print_message_name(uint8_t msg_id);

/**
 * @brief prints the configuration information
 *
 * @param Array of configuration data
 */
void print_config(uint8_t* config_to_send);

/**
 * @brief Prints the message
 *
 * @param message
 * @param message length
 */
void print_message(uint8_t* message, uint8_t len);

/**
 * @brief gets the length of the messages that will be sent.
 *
 * @param configuration array
 *
 * @return length of the message
 */
uint8_t get_data_len(uint8_t* config_to_send); // done
        
#if defined(ARDUINO_TEENSY36)
    #include "TSPISlave.h"
    
    void spi_msg(TSPISlave, uint16_t *buffer );

//=======================================

#elif defined(ARDUINO_TEENSY41)
    #include "SPISlave_T4.h"
    
    /**
     * @brief contains info for the SPI handler.  Needs to be static for use in the interrupt driven callback.
     */
    namespace static_spi_handler
    {
        /**
         * @brief Sends and receives data over SPI with the Teensy as the peripheral 
         *
         * @param SPI object
         * @param Config array
         * @param Exo data 
         *
         * @return Debug value
         */
        uint8_t peripheral_transaction(SPISlave_T4<&SPI, SPI_8_BITS> my_spi, uint8_t* config_to_send, ExoData* data); // done
        
        /**
         * @brief Sends and receives data over SPI with the Teensy as the peripheral 
         *
         * @param SPI object
         * @param Config array
         * @param Exo data 
         * @param Location to store the incoming cmd
         * @param Location to store the incoming data
         * @param The length of the array to store the incoming data
         * @param Whether to parse the message here (true), or externally (false)
         *
         * @return Debug value
         */
        uint8_t peripheral_transaction(SPISlave_T4<&SPI, SPI_8_BITS> my_spi, uint8_t* config_to_send, ExoData* data, uint8_t* cmd, uint8_t* controller_message, uint8_t msg_len, bool do_parse);
        
        const uint8_t padding = spi_data_idx::base_idx_cnt + 1;/**< the amount of padding to give the message */ //one extra for length, one for cmd, one for the end padding
        
        /**
         * @brief Packs floats bytes and places into the message 
         *
         * @param Message to place the float
         * @param Index to place the float
         * @param Value to place
         */
        void pack_float(uint8_t* message, uint8_t start_idx, float val);  // done
        
        /**
         * @brief Unpacks float from the message
         *
         * @param Message containing the value
         * @param Index where the float bytes start
         *
         * @return The unpacked float
         */
        float unpack_float(uint8_t* message, uint8_t start_idx);  // done
        
        
        /**
         * @brief Packs the config into a message format
         *
         * @param Place to store the message
         * @param Config array
         * @param Message length
         */
        void pack_config(uint8_t* peripheral_message, uint8_t* config_to_send, uint8_t msg_len); // done
        
        /**
         * @brief Packs the Exo data into a message format
         *
         * @param Place to store the message
         * @param Controller command
         * @param Config array
         * @param ExoData
         * @param Message length
         */
        void pack_data(uint8_t* peripheral_message, uint8_t cmd, uint8_t* config_to_send, ExoData* data, uint8_t msg_len); // done
        
        /**
         * @brief Takes the controller message and places the data in the appropriate location
         *
         * @param Controller message
         * @param Controller command
         * @param ExoData to place the data
         */
        void parse_message(uint8_t* controller_message, uint8_t command, ExoData* data);  // done
        
        /**
         * @brief Extracts the command from the controller message
         *
         * @param Controller message
         *
         * @return Controller command
         */
        uint8_t read_command(uint8_t* controller_message);  // done
        
        /**
         * @brief Reads the SPI and places the data in the controller message
         *
         * @param SPI object
         * @param place to store the incoming data
         */
        void read_message(SPISlave_T4<&SPI, SPI_8_BITS> my_spi, uint8_t* controller_message); // done
        
        /**
         * @brief Clears all the bytes in the provided array
         *
         * @param controller message
         */
        void clear_message(uint8_t* controller_message); // done
        
        /**
         * @brief Prints out the incoming message
         *
         * @param Controller message
         * @param ExoData
         * @param Controller command
         */
        void print_message(uint8_t* controller_message, ExoData* data, uint8_t command); // done
        
        /**
         * @brief Sets a flag that there is an unread message.  Not sure why I made this as it seems totally unnecessary
         *
         * @param Location of flag
         */
        void set_message_flag(bool* is_unread_message);
        
        /**
         * @brief Prints the debug location
         *
         * @param Debug location value
         */
        void print_debug(uint16_t debug_location);
        
        
        /**
         * @brief Unpacks an incoming null message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_null_cmd(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming test message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_test_cmd(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming send config message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_send_config(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming send data message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_send_data_exo(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming update controller message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_update_controller(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming update controller parameter message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_update_controller_params(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming calibrate torque sensor message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_calibrate_torque_sensor(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming calibrate FSR message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_calibrate_fsr(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming refine FSR message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_refine_fsr(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming motor enable/disable message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_motor_enable_disable(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming motor zero message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_motor_zero(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming update status message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_update_status(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming update controller parameter workaround message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_update_controller_params_workaround(uint8_t* controller_message, ExoData* data);
        
        /**
         * @brief Unpacks an incoming calibrate FSR workaround message
         *
         * @param Controller Message
         * @param ExoData
         */
        void unpack_calibrate_fsr_workaround(uint8_t* controller_message, ExoData* data);
        
        
        
        // debug locations
        const uint8_t debug_entry_bit = 0; /**< Bit to set when the function is entered */
        const uint8_t debug_sent_length_bit = 1; /**< Bit to set when the length is sent */
        const uint8_t debug_send_config_bit = 2; /**< Bit to set when the config sent */
        const uint8_t debug_send_data_bit = 3; /**< Bit to set when data is sent */
        const uint8_t debug_q_bit = 4; /**< Placeholder bit to set */
        const uint8_t debug_w_bit = 5; /**< Placeholder bit to set */
        const uint8_t debug_e_bit = 6; /**< Placeholder bit to set */
        const uint8_t debug_r_bit = 7; /**< Placeholder bit to set */
        const uint8_t debug_t_bit = 8; /**< Placeholder bit to set */
        const uint8_t debug_y_bit = 9; /**< Placeholder bit to set */
        const uint8_t debug_u_bit = 10; /**< Placeholder bit to set */
        const uint8_t debug_i_bit = 11; /**< Placeholder bit to set */
        const uint8_t debug_o_bit = 12; /**< Placeholder bit to set */
        const uint8_t debug_p_bit = 13; /**< Placeholder bit to set */
        const uint8_t debug_a_bit = 14; /**< Placeholder bit to set */
        const uint8_t debug_exit_bit = 15; /**< Bit to set when exiting the function */
        
    };
    
    // void spi_msg(SPISlave_T4<&SPI, SPI_8_BITS>, uint8_t buffer_len, uint8_t *config, uint8_t &buffer );

#elif defined(ARDUINO_ARDUINO_NANO33BLE)
    #include <SPI.h>
    
    /**
     * @brief Class to handle the SPI messages
     *
     * Need to have SPI enabled in ino
     */
    class SPIHandler
    {
        public:
            
            SPIHandler(uint8_t* config_to_send, ExoData* exo_data);
            
            /**
             * @brief send and receive data based on the command
             *
             * @param command referring to what data to send
             */
            void transaction(uint8_t command); // done
            
            /**
             * @brief send and receive data based on the command
             *
             * @param command referring to what data to send
             * @param joint id that the message refers to.
             */
            void transaction(uint8_t command, uint8_t joint_id); // done
            
        private:
            //SPISlave_T4<&SPI, SPI_8_BITS> _my_spi;
            uint8_t _data_len; /**< Length of the data */
            static const uint8_t _padding = spi_data_idx::base_idx_cnt + 1;/**< Padding to add to the message *///one extra for length, one for cmd, one for the end padding
            uint8_t _msg_len; /**< length of the message */
            uint8_t _joint_id; /**< Joint id the message refers to */
            uint8_t _controller_message[_padding+spi_cmd::max_data_len+spi_data_idx::is_ff::num_bytes];  /**< Array to contain the controller message*/ // Should be largest of parameters, data, and config length.
            uint8_t _peripheral_message[_padding+spi_cmd::max_data_len+spi_data_idx::is_ff::num_bytes]; /**< Array to contain the peripheral message*/ //, ini_config::number_of_keys)]; // use which is bigger, this will almost always be the data length.
            uint8_t _command; /**< command for the message to send */
            uint8_t* _config_to_send;  /**< pointer to the config array */ // holds and array.
            
            
            /**
             * @brief gets the length of the data based on the config
             *
             * @param Config array
             *
             * @return Length of the data
             */
            static uint8_t _get_data_len(uint8_t *config_to_send); 
            
            /**
             * @brief Packs floats into a message
             *
             * @param Message to place the float
             * @param Location to place the float in the message
             * @param Value to place
             *
             */
            static void _pack_float(uint8_t* message, uint8_t start_idx, float val);

            /**
             * @brief Unpacks a float from a message
             *
             * @param Message containing the float
             * @param Location where the float starts
             *
             * @return unpacked value
             */
            static float _unpack_float(uint8_t* message, uint8_t start_idx);  
            
            /**
             * @brief Parse the incoming message.
             */
            void _parse_message(); //done
            
            /**
             * @brief parse incoming config info
             */
            void _parse_config(); // done
            
            /**
             * @brief parse incoming data
             */
            void _parse_data();  // done
            
            /**
             * @brief pack the controller message
             */
            void _pack_message(); // done
            
            /**
             * @brief send the controller message
             */
            void _send_message(); // done
            
            /**
             * @brief clear the message
             */
            void _clear_message(); // done
            
            
            /**
             * @brief pack the null command message
             */
            void _pack_null_cmd(); 
            
            /**
             * @brief pack the test command message
             */
            void _pack_test_cmd();
            
            /**
             * @brief pack the send config message
             */
            void _pack_send_config(); 
            
            /**
             * @brief pack the send exo data message
             */
            void _pack_send_data_exo(); 
            
            /**
             * @brief pack the update controller message
             */
            void _pack_update_controller(); 
            
            /**
             * @brief pack the update controller parameter message
             */
            void _pack_update_controller_params(); 
            
            /**
             * @brief pack the calibrate torque sensor message
             */
            void _pack_calibrate_torque_sensor(); 
            
            /**
             * @brief pack the calibrate fsr message
             */
            void _pack_calibrate_fsr(); 
            
            /**
             * @brief pack the refine fsr message
             */
            void _pack_refine_fsr(); 
            
            /**
             * @brief pack the motor enable/disable message
             */
            void _pack_motor_enable_disable(); 
            
            /**
             * @brief pack the motor zero message
             */
            void _pack_motor_zero(); 
            
            /**
             * @brief pack the update status message
             */
            void _pack_update_status(); 
            
            /**
             * @brief pack the update controller workaround message
             */
            void _pack_update_controller_params_workaround(); 
            
            /**
             * @brief pack the calibrate fsr message
             */
            void _pack_calibrate_fsr_workaround(); 
            
            ExoData* _data;  /**<pointer to ExoData that is getting updated by SPI so they share memory.*/
        
    };
    
    // void spi_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, uint8_t *buffer);

#endif

#endif


