
#ifndef Config_h
#define Config_h 

    #define AK_Board_V0_1 1
    #define AK_Board_V0_3 2
    #define AK_Board_V0_4 3

    // TODO : Incorporate into parse INI
    #define BOARD_VERSION AK_Board_V0_4  
    
    #define LOOP_FREQ_HZ 500
    #define LOOP_TIME_TOLERANCE 0.1 
    
    #define USE_SPEED_CHECK 1 
    
    namespace sync_time
    {
        const unsigned int NUM_START_STOP_BLINKS = 1;  // the number of times to have the LED on during the start stop sequence
        const unsigned int SYNC_HALF_PERIOD_US = 125000;  // half blink period in micro seconds
        const unsigned int SYNC_START_STOP_HALF_PERIOD_US = 4 * SYNC_HALF_PERIOD_US; // Half blink period for the begining and end of the sequence.  This is usually longer so it is easy to identify.
    }
    
    namespace torque_calibration
    {
        const float AI_CNT_TO_V = 3.3 / 4096; // conversion from count to voltage
        const float TRQ_V_TO_NM = -42.500; // conversion from voltage to Nm (Negative do to mismatch in torque sensor and motor torque directions)
    }

    namespace BLE_times
    {
        const float _status_msg_delay = 1000000; //microseconds
        const float _real_time_msg_delay = 40000; //microseconds (20000)
        const float _update_delay = 100000; //microseconds
    }
    
    namespace SPI_times
    {
        const float UPDATE_PERIOD= 10000000;//10000; //microseconds, time between updating data over spi
    }
#endif