#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER

#define BOARD_VERSION AK_Board_V0_1

#if BOARD_VERSION == AK_Board_V0_1
    
    #if defined(ARDUINO_TEENSY36)
    #include "Arduino.h"
    namespace logic_micro_pins  //teensy
    {
        // Serial Pins, NC
        const unsigned int rx1_pin = 0;
        const unsigned int tx1_pin = 1;
        
        // CAN Pins
        const unsigned int can_rx_pin = 4;
        const unsigned int can_tx_pin = 3;
        
        // FSR Pins
        const unsigned int fsr_sense_left_heel_pin = A14;
        const unsigned int fsr_sense_left_toe_pin = A15;
        const unsigned int fsr_sense_right_heel_pin= A7;
        const unsigned int fsr_sense_right_toe_pin = A6;
        
        // Torque Sensor Pins
        const unsigned int num_available_joints = 2;
        const unsigned int torque_sensor_left[] = {A17, A16};
        //const unsigned int torque_sensor_left1 = A16;
        const unsigned int torque_sensor_right[] = {A9, A8};
        //const unsigned int torque_sensor_right1 = A8;
        
        
        // Sync LED Pins
        const unsigned int sync_led_pin = 29;
        const unsigned int sync_default_pin = 25;
        
        // Status LED Pins
        const unsigned int status_led_r_pin= 28;
        const unsigned int status_led_g_pin = 27;
        const unsigned int status_led_b_pin = 26;
        
        // SPI Follower Pins
        const unsigned int miso_pin = 12;
        const unsigned int mosi_pin= 11;
        const unsigned int sck_pin = 13;
        const unsigned int cs_pin = 10;
        const unsigned int spi_mode = 16;
        
        // Pin to Stop the Motors
        const unsigned int motor_stop_pin = 6;
        
        // Pin to use when we need a value but don't actually want to use it.
        const unsigned int not_connected_pin = 42;  // selected 42 as it is a pad on the back so I figure it won't hurt anything if something goes wrong.
    };
    
    #elif defined(ARDUINO_ARDUINO_NANO33BLE)
    namespace coms_micro_pins  //nano
    {
        
    };
    #endif
#endif

#endif
