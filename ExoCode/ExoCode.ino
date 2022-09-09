/*
   Code used to run the exo from the teensy.  This communicates with the nano over SPI.

   P. Stegall Jan 2022
*/
#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)

#define INCLUDE_FLEXCAN_DEBUG  // used to print CAN Debugging messages for the motors.
//#define MAKE_PLOTS  // Do prints for plotting when uncommented.
#define MAIN_DEBUG   // Print Arduino debugging statements when uncommented.
//#define HEADLESS // used when there is no app access.

// Standard Libraries
#include <stdint.h>
#include <IntervalTimer.h>

// for the include files we can eventually create a library properties files but right now just providing the path should work.
// Common Libraries
#include "src\Board.h"
#include "src\ExoData.h"
#include "src\Exo.h"
#include "src\Utilities.h"
#include "src\StatusDefs.h"
#include "src\ComsMCU.h"

// Specific Libraries
#include "src\ParseIni.h"
#include "src\ParamsFromSD.h"

// can remove these if we don't end up using SPI
#if defined(ARDUINO_TEENSY36)
  #include <TSPISlave.h>
#elif defined(ARDUINO_TEENSY41)
  #include "SPISlave_T4.h"
#endif
//#include "src\Motor.h"

// Array used to store config information
namespace config_info
{
    uint8_t (config_to_send)[ini_config::number_of_keys];
}

// I don't like having this here but I was having an issue with the spi object and functions in the callback having the right scope.
// setup components needed for SPI, can be removed if we don't end up using SPI
namespace spi_peripheral
{
    SPISlave_T4<&SPI, SPI_8_BITS> my_spi;
    ExoData* data;// need to set this pointer after data object created with: spi_peripheral::data = &exo_data;
    bool is_unread_message = false; // flag used so the main loop knows if there is a message to parse.
    uint8_t debug_location; // used to track location the code went without using print statements.

    const uint8_t max_msg_len = static_spi_handler::padding + spi_cmd::max_data_len+spi_data_idx::is_ff::num_bytes;  // Should be largest of parameters, data, and config length.
    uint8_t msg_len = max_msg_len;
    uint8_t controller_message[max_msg_len] = {0}; // stores the message from the controller

    uint8_t cmd = 0; // command sent from the controller.
    bool do_parse_in_callback = false;  // Flag for parsing the message in the callback or main loop.
    
    // Function called when the SPI used.
    void spi_callback()
    {
        
        msg_len = static_spi_handler::padding + max(max(spi_cmd::max_param_len, get_data_len(config_info::config_to_send)), ini_config::number_of_keys) + spi_data_idx::is_ff::num_bytes;
//        data->left_leg.hip.motor.p_des--;
        debug_location = static_spi_handler::peripheral_transaction(my_spi, config_info::config_to_send, data, &cmd, controller_message, msg_len, do_parse_in_callback);
        is_unread_message = true;
        return;
    }
    
    // function used to parse the message if done outside of callback.
    void spi_handle_message()
    {
        static_spi_handler::parse_message(controller_message, cmd, data);
      
    }
}

void setup()
{
  Serial.begin(115200);
//  TODO: Remove serial while for deployed version as this would hang
    while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
    }

    // get the config information from the SD card.
    ini_parser(config_info::config_to_send);
    
    // Print to confirm config came through correctly.  Should not contain zeros.
    #ifdef MAIN_DEBUG
        for (int i = 0; i < ini_config::number_of_keys; i++)
        {
          Serial.print("[");
          Serial.print(i);
          Serial.print("] : ");
          Serial.print((int)config_info::config_to_send[i]);
          Serial.print("\n");
        }
        Serial.print("\n");
    #endif
    
    // labels for the signals if plotting.
    #ifdef MAKE_PLOTS
          Serial.print("Left_hip_trq_cmd, ");
          Serial.print("Left_hip_current, ");
          Serial.print("Right_hip_trq_cmd, ");
          Serial.print("Right_hip_current, ");
          Serial.print("Left_ankle_trq_cmd, ");
          Serial.print("Left_ankle_current, ");
          Serial.print("Right_ankle_trq_cmd, ");
          Serial.print("Right_ankle_current, ");
          Serial.print("\n");
      #endif
  
}



void loop()
{
    // check if the main loop is still running.
    #ifdef MAIN_DEBUG
        static unsigned int loop_counter = 0;
        Serial.print("Superloop :: loop counter = ");
        Serial.println(loop_counter++);
    #endif

    
    
    static bool first_run = true;
    
    // create the data and exo objects
    static ExoData exo_data(config_info::config_to_send);

    #ifdef MAIN_DEBUG
        if (first_run)
        {
            Serial.println("Superloop :: exo_data created"); 
        }
    #endif
    static Exo exo(&exo_data);
    #ifdef MAIN_DEBUG
        if (first_run)
        {
            Serial.println("Superloop :: exo created");
        }
    #endif
    
    if (first_run)
    {
        first_run = false; 
        #ifdef MAIN_DEBUG
            Serial.println("Superloop :: Start First Run Conditional");
            Serial.print("Superloop :: exo_data.left_leg.hip.is_used = ");
            Serial.print(exo_data.left_leg.hip.is_used);
            Serial.print("\n");
            Serial.print("Superloop :: exo_data.right_leg.hip.is_used = ");
            Serial.print(exo_data.right_leg.hip.is_used);
            Serial.print("\n");
            Serial.print("Superloop :: exo_data.left_leg.knee.is_used = ");
            Serial.print(exo_data.left_leg.knee.is_used);
            Serial.print("\n");
            Serial.print("Superloop :: exo_data.right_leg.knee.is_used = ");
            Serial.print(exo_data.right_leg.knee.is_used);
            Serial.print("\n");
            Serial.print("Superloop :: exo_data.left_leg.ankle.is_used = ");
            Serial.print(exo_data.left_leg.ankle.is_used);
            Serial.print("\n");
            Serial.print("Superloop :: exo_data.right_leg.ankle.is_used = ");
            Serial.print(exo_data.right_leg.ankle.is_used);
            Serial.print("\n");
            Serial.print("\n");
        #endif

        // point the callback to exo_data
        spi_peripheral::data = &exo_data;
        #ifdef MAIN_DEBUG
            Serial.println("Superloop :: SPI Data pointer updated");
        #endif
        // connect the callback
        spi_peripheral::my_spi.onReceive(spi_peripheral::spi_callback);        
        #ifdef MAIN_DEBUG
            Serial.println("Superloop :: SPI callback set");
        #endif
        // start the SPI listening
        spi_peripheral::my_spi.begin();
        #ifdef MAIN_DEBUG
            Serial.println("Superloop :: SPI Begin");
        #endif
        
        // debug to check the message is coming through
        exo_data.left_leg.hip.motor.p_des = 300;
        
        // Only make calls to used motors.
        if (exo_data.left_leg.hip.is_used)
        {
            // turn motor on
            exo_data.left_leg.hip.motor.is_on = true;
            exo.left_leg._hip._motor->on_off();
            // make sure gains are 0 so there is no funny business.
            exo_data.left_leg.hip.motor.kp = 0;
            exo_data.left_leg.hip.motor.kd = 0;
            #ifdef HEADLESS
                exo.left_leg._hip._motor->zero();
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Left Hip Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.left_leg.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data);
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Left Hip Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.left_leg.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque; // start in zero torque
                exo.left_leg._hip.set_controller(exo_data.left_leg.hip.controller.controller);
            #endif
        }
        
        if (exo_data.right_leg.hip.is_used)
        {
            exo_data.right_leg.hip.motor.is_on = true;
            exo.right_leg._hip._motor->on_off();
            exo_data.right_leg.hip.motor.kp = 0;
            exo_data.right_leg.hip.motor.kd = 0;
            #ifdef HEADLESS
                exo.right_leg._hip._motor->zero();
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Right Hip Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.right_leg.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data);
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Right Hip Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.right_leg.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque; // start in zero torque
                exo.right_leg._hip.set_controller(exo_data.right_leg.hip.controller.controller);
            #endif
        }

        if (exo_data.left_leg.ankle.is_used)
        {
            #ifdef MAIN_DEBUG
              Serial.println("Superloop :: Left Ankle Used");
            #endif
            exo_data.left_leg.ankle.motor.is_on = true;
            exo.left_leg._ankle._motor->on_off();
            exo_data.left_leg.ankle.motor.kp = 0;
            exo_data.left_leg.ankle.motor.kd = 0;
            #ifdef HEADLESS
                exo.left_leg._ankle._motor->zero();
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Left Ankle Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.left_leg.ankle.id, config_info::config_to_send[config_defs::exo_ankle_default_controller_idx], 0, &exo_data);
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Left Ankle Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.left_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque; // start in zero torque
                exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);              
            #endif
        }
        
        if (exo_data.right_leg.ankle.is_used)
        {
            exo_data.right_leg.ankle.motor.is_on = true;
            exo.right_leg._ankle._motor->on_off();
            exo_data.right_leg.ankle.motor.kp = 0;
            exo_data.right_leg.ankle.motor.kd = 0;
            #ifdef HEADLESS
                exo.right_leg._ankle._motor->zero();
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Right Ankle Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.right_leg.ankle.id, config_info::config_to_send[config_defs::exo_ankle_default_controller_idx], 0, &exo_data);
                #ifdef MAIN_DEBUG
                  Serial.println("Superloop :: Right Ankle Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.right_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque; // start in zero torque
                exo.right_leg._ankle.set_controller(exo_data.right_leg.ankle.controller.controller);
            #endif
        }
        
        // give the motors time to wake up.  Can eventually be removed when using non damaged motors.
        #ifdef MAIN_DEBUG
          Serial.println("Superloop :: Motor Charging Delay - Please be patient");
        #endif 
        exo_data.status = status_defs::messages::motor_start_up; 
        unsigned int motor_start_delay_ms = 10;//60000;
        unsigned int motor_start_time = millis();
        unsigned int dot_print_ms = 1000;
        unsigned int last_dot_time = millis();
        while (millis() - motor_start_time < motor_start_delay_ms)
        {
            exo.status_led.update(exo_data.status);
            #ifdef MAIN_DEBUG
              if(millis() - last_dot_time > dot_print_ms)
              {
                last_dot_time = millis();
                Serial.print(".");
              }
              
            #endif
        }
        #ifdef MAIN_DEBUG
          Serial.println();
        #endif

        // Configure the system if you can't set it with the app
        #ifdef HEADLESS
            bool enable_overide = true;
            if(exo_data.left_leg.hip.is_used)
            {
                exo_data.left_leg.hip.calibrate_torque_sensor = true; 
                exo_data.left_leg.hip.motor.enabled = true;
                exo.left_leg._hip._motor->enable(enable_overide);
            }
           
            if(exo_data.right_leg.hip.is_used)
            {
                exo_data.right_leg.hip.calibrate_torque_sensor = true; 
                exo_data.right_leg.hip.motor.enabled = true;
                exo.right_leg._hip._motor->enable(enable_overide);
            }
            if(exo_data.left_leg.ankle.is_used)
            {
                exo_data.left_leg.ankle.calibrate_torque_sensor = true; 
                exo_data.left_leg.ankle.motor.enabled = true;
                exo.left_leg._ankle._motor->enable(enable_overide);
            }
           
            if(exo_data.right_leg.ankle.is_used)
            {
                exo_data.right_leg.ankle.calibrate_torque_sensor = true;  
                exo_data.right_leg.ankle.motor.enabled = true;
                exo.right_leg._ankle._motor->enable(enable_overide);
            }
        #endif
 
        #ifdef MAIN_DEBUG
            #ifdef HEADLESS
                Serial.println("Superloop :: Motors Enabled");
                Serial.println("Superloop :: Parameters Set");
            #endif
            Serial.println("Superloop :: End First Run Conditional");
        #endif
    }

    // run the calibrations we need to do if not set through the app.
    #ifdef HEADLESS
        static bool static_calibration_done = false;
        unsigned int pause_after_static_calibration_ms = 10000;
        static unsigned int time_dynamic_calibration_finished; 
        static bool pause_between_calibration_done = false;   
        static bool dynamic_calibration_done = false;
    
        
        // do data plotting
        static float old_time = micros();
        float new_time = micros();
        if(new_time - old_time > 500000 && dynamic_calibration_done)
        {
            #ifdef MAKE_PLOTS
                Serial.print(exo_data.left_leg.hip.motor.t_ff);
                Serial.print(", ");
                Serial.print(exo_data.left_leg.hip.motor.i);
                Serial.print(", ");
                Serial.print(exo_data.right_leg.hip.motor.t_ff);
                Serial.print(", ");
                Serial.print(exo_data.right_leg.hip.motor.i);
                Serial.print(", ");
                Serial.print(exo_data.left_leg.ankle.motor.t_ff);
                Serial.print(", ");
                Serial.print(exo_data.left_leg.ankle.motor.i);
                Serial.print(", ");
                Serial.print(exo_data.right_leg.ankle.motor.t_ff);
                Serial.print(", ");
                Serial.print(exo_data.right_leg.ankle.motor.i);
                Serial.print("\n");
            #endif
    
            
        }
    
        
        // do torque sensor calibration
        if ((!static_calibration_done) && (!exo_data.left_leg.ankle.calibrate_torque_sensor && !exo_data.right_leg.ankle.calibrate_torque_sensor))
        {
            #ifdef MAIN_DEBUG
              Serial.println("Superloop : Static Calibration Done");
            #endif
            static_calibration_done  = true;
            time_dynamic_calibration_finished = millis();
            exo_data.status = status_defs::messages::test;
        }
    
        // pause between static and dynamic calibration so we have time to start walking
        if (!pause_between_calibration_done && (static_calibration_done && ((time_dynamic_calibration_finished +  pause_after_static_calibration_ms) < millis() ))) 
        {
            #ifdef MAIN_DEBUG
              Serial.println("Superloop : Pause Between Calibration Finished");
            #endif
            if(exo_data.left_leg.is_used)
            {
                exo_data.left_leg.do_calibration_toe_fsr = true;
                exo_data.left_leg.do_calibration_refinement_toe_fsr = true;
                exo_data.left_leg.do_calibration_heel_fsr = true;
                exo_data.left_leg.do_calibration_refinement_heel_fsr = true;
            }
           
            if(exo_data.right_leg.is_used)
            {
                exo_data.right_leg.do_calibration_toe_fsr = true;
                exo_data.right_leg.do_calibration_refinement_toe_fsr = true;
                exo_data.right_leg.do_calibration_heel_fsr = true;
                exo_data.right_leg.do_calibration_refinement_heel_fsr = true;  
            }
            pause_between_calibration_done = true;
        }
            
        // do the dynamic calibrations
        if ((!dynamic_calibration_done) && (pause_between_calibration_done) && (!exo_data.left_leg.do_calibration_toe_fsr && !exo_data.left_leg.do_calibration_refinement_toe_fsr && !exo_data.left_leg.do_calibration_heel_fsr && !exo_data.left_leg.do_calibration_refinement_heel_fsr))
        {
            #ifdef MAIN_DEBUG
                Serial.println("Superloop : Dynamic Calibration Done");
            #endif
            
            if (exo_data.left_leg.hip.is_used)
            {
                exo_data.left_leg.hip.controller.controller = config_info::config_to_send[config_defs::exo_hip_default_controller_idx];
                exo.left_leg._hip.set_controller(exo_data.left_leg.hip.controller.controller);
                #ifdef MAIN_DEBUG
                    Serial.println("Superloop : Left Hip Controller Set");
                #endif
            }
            
            if (exo_data.right_leg.hip.is_used)
            {
                exo_data.right_leg.hip.controller.controller = config_info::config_to_send[config_defs::exo_hip_default_controller_idx];
                exo.right_leg._hip.set_controller(exo_data.right_leg.hip.controller.controller); 
                #ifdef MAIN_DEBUG
                    Serial.println("Superloop : Right Hip Controller Set");
                #endif
            }
            
            if (exo_data.left_leg.ankle.is_used)
            {
                exo_data.left_leg.ankle.controller.controller = config_info::config_to_send[config_defs::exo_ankle_default_controller_idx];
                exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);
                #ifdef MAIN_DEBUG
                    Serial.println("Superloop : Left Ankle Controller Set");
                #endif
            }
      
            if (exo_data.right_leg.ankle.is_used)
            {
                exo_data.right_leg.ankle.controller.controller = config_info::config_to_send[config_defs::exo_ankle_default_controller_idx];
                exo.right_leg._ankle.set_controller(exo_data.right_leg.ankle.controller.controller);
                #ifdef MAIN_DEBUG
                    Serial.println("Superloop : Right Ankle Controller Set");
                #endif
            }
            
            dynamic_calibration_done = true;
          
        }
    #endif                                                                                        

    // do exo calculations
    
    exo.run();
    
    // print the exo_data at a fixed period.
//    unsigned int data_print_ms = 5000;
//    static unsigned int last_data_time = millis();
//    if(millis() - last_data_time > data_print_ms)
//    {
//        Serial.println("\n\n\nSuperloop :: Timed print : ");
//        exo_data.print();
//        last_data_time = millis();
//    }
    
    // Print some dots so we know it is doing something
    #ifdef MAIN_DEBUG
        unsigned int dot_print_ms = 5000;
        static unsigned int last_dot_time = millis();
        if(millis() - last_dot_time > dot_print_ms)
        {
          last_dot_time = millis();
          Serial.print(".");
        }

       
    #endif 
    
    // When there is a new message process it.
    if(spi_peripheral::is_unread_message)
    {
        // Not doing the parsing in the transaction so do it here.
        if(!spi_peripheral::do_parse_in_callback)
        {
            spi_peripheral::spi_handle_message();
        }
        #ifdef MAIN_DEBUG
            Serial.println("\n\n\nSuperloop :: New Message : ");
            exo_data.print();
            
        #endif

        spi_peripheral::is_unread_message = false;
    }
}


#elif defined(ARDUINO_ARDUINO_NANO33BLE)  // board name is ARDUINO_[build.board] property in the board.txt file here found at C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\hardware\mbed_nano\2.6.1  They just already prepended it with ARDUINO so you have to do it twice.
#include <stdint.h>
#include "src/ParseIni.h"
#include "src/ExoData.h"
#include "src/ComsMCU.h"

// create array to store config.
namespace config_info
{
    uint8_t (config_to_send)[ini_config::number_of_keys];
}

// dummy config to use when not getting it from the teensy
//namespace config_info
//{
//    uint8_t (config_to_send)[ini_config::number_of_keys] = {
//      1,
//      2,
//      3,
//      1,
//      2,
//      1,
//      3,
//      1,
//      1,
//      1,
//      1,
//      1,
//      1,
//    };
//}

void setup()
{
    Serial.begin(115200);
    while (!Serial);
}

void loop()
{
    static ExoData* exo_data = new ExoData(config_info::config_to_send);
    static ComsMCU* mcu = new ComsMCU(exo_data, config_info::config_to_send);
    mcu->handle_ble();
    mcu->local_sample();
    mcu->update_spi();
    mcu->update_gui();
}

#else // code to use when microcontroller is not recognized.
void setup()
{
  Serial.begin(115200);
  //TODO: Remove serial while for deployed version as this would hang
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.print("Unknown Microcontroller");
  Serial.print("\n");
}

void loop()
{

}


#endif
