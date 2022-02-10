/*
 * Code used to run the exo from the teensy.  This communicates with the nano over SPI.
 * 
 * P. Stegall Jan 2022
 */

#if defined(ARDUINO_TEENSY36)  

    // Standard Libraries
    #include <stdint.h>
    #include <IntervalTimer.h>
    
    
    // for the include files we can eventually create a library properties files but right now just providing the path should work.
    // Common Libraries
    #include "src\board.h"
    #include "src\ExoData.h"
    #include "src\Exo.h"
    
    // Specific Librarys
    #include "src\parseIni.h"
    #include "src\Sync_Led.h"
    #include "src\Status_Led.h"
    #include "src\TSPISlave.h"
    
    
    namespace led{
        IntervalTimer syncTimer;  // Create a timer for setting for handling the interupt, this is needed as the ISR cannot access class variables since you cannot pass a self to the ISR.
        Sync_Led syncLed(logic_micro_pins::sync_led_pin, SYNC_START_STOP_HALF_PERIOD_US, SYNC_HALF_PERIOD_US, SYNC_LED_ON_STATE, logic_micro_pins::sync_default_pin);  // Create a sync LED object, the first and last arguments (pin) are found in Board.h, and the rest are in Sync_Led.h.  If you do not have a digital input for the default state you can remove SYNC_DEFAULT_STATE_PIN.  
        Status_Led statusLed(logic_micro_pins::status_led_r_pin, logic_micro_pins::status_led_g_pin, logic_micro_pins::status_led_b_pin);  // Create the status LED object.  
    
       /* 
        *  The interupt service routine
        *  This is terrible but integrating the timer into the class doesn't work well, as the callback can't access member variables or functions.
        *  
        *  PS 2021.10
        */
        void grossLedInteruptWrapper(void)
        {
            led::syncLed.syncLedHandler(); // calculate the LED state based on the timer, but don't change the actual LED state.
            led::syncTimer.begin(led::grossLedInteruptWrapper, led::syncLed.currentSyncPeriod);  // update the timer period ideally we would only do this if it changed, might add a flag to syncLed if needed
        }
    }
    // this creates a does not name a type error not sure why as it worked before.  Could be an issue with the lib.  Commenting out for now.
    //led::syncTimer.begin(Led::grossLedInteruptWrapper, Led::syncLed.currentSyncPeriod);
    
    
    namespace config_info
    {
        uint8_t (config_to_send)[ini_config::number_of_keys];
        
    }

    
    void callback()//executed every 2ms
    {
    
      
    }
    
    void setup()
    {
        
        Serial.begin(115200);
        //TODO: Remove serial while for deployed version as this would hang
        while (!Serial) {
            ; // wait for serial port to connect. Needed for native USB
        } 
        Serial.println("Teensy Microcontroller");


        ini_parser(config_info::config_to_send);
        //uint8_t (config_to_send)[ini_config::number_of_keys];
        
        Serial.println();
        Serial.println("setup : Coded Config Data");
        for (int i = 0; i<ini_config::number_of_keys; i++)
        {
            Serial.print("[");
            Serial.print(i);
            Serial.print("] : ");
            Serial.println((int)config_info::config_to_send[i]);
            
        }
        Serial.println();

        Serial.println();
        Serial.println(static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_info::config_to_send[config_defs::exo_side_idx]);
    
        // Now that we have read the config file create the data structure and exoskeleton object.
       
    }
    
    
    void loop()
    {
        static ExoData exo_data(config_info::config_to_send);
        static Exo exo(&exo_data);
         
        //Led::syncLed.updateLed();  // actually change the led state, this also updates ledIsOn for recording the actual on/off state 
        
        // Need to update this for the new bluetooth stream
//        if (led::syncLed.doBlink | led::syncLed.doStartStopSequence) //if we are within a trial stream data
//        {
//          //stream = 1; 
//        }
//        else 
//        {
//          //stream = 0;
//        }

          /* Temp code to test the FSR, need to move them to public in leg.h */
          //+++++++++++++++++++++++++++++++++++++++++
          static bool do_heel_cal = true;
          static bool do_toe_cal = true;
          
          static bool heel_cal_done = false;
          static bool toe_cal_done = false;
          
          static bool do_heel_cal_refinement = false;
          static bool do_toe_cal_refinement = false;
          
          // once the calibration is done do the refinement.
          if (!do_heel_cal & !heel_cal_done)
          {
              heel_cal_done = true;
              do_heel_cal_refinement = true;
          }

          if (!do_toe_cal & !toe_cal_done)
          {
              toe_cal_done = true;
              do_toe_cal_refinement = true;
          }
          
          do_heel_cal = exo.left_leg._heel_fsr.calibrate(do_heel_cal);
          do_toe_cal = exo.left_leg._toe_fsr.calibrate(do_toe_cal);

          do_heel_cal_refinement = exo.left_leg._heel_fsr.refine_calibration(do_heel_cal_refinement);
          do_toe_cal_refinement = exo.left_leg._toe_fsr.refine_calibration(do_toe_cal_refinement);


          if(!do_toe_cal & !do_toe_cal_refinement & !do_heel_cal & !do_heel_cal_refinement)
          {
              Serial.print("toe reading : \t");
              Serial.print(exo.left_leg._toe_fsr.read());
              Serial.print("\t heel reading : \t");
              Serial.println(exo.left_leg._heel_fsr.read());
          }


          

          

          
          //+++++++++++++++++++++++++++++++++++++++++

  
    
    
      
    }

#elif defined(ARDUINO_ARDUINO_NANO33BLE)  // board name is ARDUINO_[build.board] property in the board.txt file here found at C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\hardware\mbed_nano\2.6.1  They just already prepended it with ARDUINO so you have to do it twice.
    #include "src\parseIni.h"
    //#include "src\ExoData.h"
    #include <stdint.h>
    
    void setup()
    {
        Serial.begin(115200);
        //TODO: Remove serial while for deployed version as this would hang
        while (!Serial) {
            ; // wait for serial port to connect. Needed for native USB
        } 
        Serial.println("Nano Microcontroller");

        // something to work with till the SPI works.
        uint8_t test_config[] = {1, 1, 3, 1, 2, 1, 3, 1, 1, 1};

        Serial.println();
        Serial.println("Coded Config Data");
        for (int i = 0; i<ini_config::number_of_keys; i++)
        {
            Serial.print("[");
            Serial.print(i);
            Serial.print("] : ");
            Serial.println((int)test_config[i]);
            
        }
        Serial.println();
    }

    void loop()
    {
        
    }
    
#else
    void setup()
    {
        Serial.begin(115200);
        //TODO: Remove serial while for deployed version as this would hang
        while (!Serial) {
            ; // wait for serial port to connect. Needed for native USB
        } 
        Serial.println("Unknown Microcontroller");
    }

    void loop()
    {
      
    }


#endif
