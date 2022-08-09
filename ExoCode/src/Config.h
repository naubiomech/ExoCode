
#ifndef Config_h
#define Config_h 

    #define AK_Board_V0_1 1
    #define AK_Board_V0_3 2
    #define AK_Board_V0_4 3

    // TODO : Incorporate into parse INI
    #define BOARD_VERSION AK_Board_V0_4  
    
    #define LOOP_FREQ_HZ 500
    
    namespace sync_time
    {
        const unsigned int NUM_START_STOP_BLINKS = 1;  // the number of times to have the LED on during the start stop sequence
        const unsigned int SYNC_HALF_PERIOD_US = 125000;  // half blink period in micro seconds
        const unsigned int SYNC_START_STOP_HALF_PERIOD_US = 4 * SYNC_HALF_PERIOD_US; // Half blink period for the begining and end of the sequence.  This is usually longer so it is easy to identify.
    } 
    
    namespace torque_calibration
    {
        const unsigned int AI_CNT_TO_V = 3.3 / 4096; 
        const unsigned int TRQ_V_TO_NM = 56.5 / (2.1);
    }
#endif