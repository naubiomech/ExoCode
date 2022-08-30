/*
   Code used to run the exo from the teensy.  This communicates with the nano over SPI.

   P. Stegall Jan 2022
*/
#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)

#define INCLUDE_FLEXCAN_DEBUG
//#define MAKE_PLOTS
#define MAIN_DEBUG

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

// Specific Librarys
#include "src\ParseIni.h"
#include "src\ParamsFromSD.h"
#if defined(ARDUINO_TEENSY36)
  #include <TSPISlave.h>
#elif defined(ARDUINO_TEENSY41)
  #include "SPISlave_T4.h"
#endif
//#include "src\Motor.h"

namespace config_info
{
uint8_t (config_to_send)[ini_config::number_of_keys];

}

void setup()
{
  Serial.begin(115200);
//  TODO: Remove serial while for deployed version as this would hang
    while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
    }

    ini_parser(config_info::config_to_send);
    
  
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
    static bool first_run = true;
    // create the data and exo objects
    static ExoData exo_data(config_info::config_to_send);
    static ComsMCU comms(&exo_data, config_info::config_to_send);
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

       
        // Only make calls to used motors.
        if (exo_data.left_leg.hip.is_used)
        {
            exo_data.left_leg.hip.motor.is_on = true;
            exo.left_leg._hip._motor->on_off();
            exo_data.left_leg.hip.motor.kp = 0;
            exo_data.left_leg.hip.motor.kd = 0;
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

        }
        
        if (exo_data.right_leg.hip.is_used)
        {
            exo_data.right_leg.hip.motor.is_on = true;
            exo.right_leg._hip._motor->on_off();
            exo_data.right_leg.hip.motor.kp = 0;
            exo_data.right_leg.hip.motor.kd = 0;
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
        }
        
        if (exo_data.right_leg.ankle.is_used)
        {
            exo_data.right_leg.ankle.motor.is_on = true;
            exo.right_leg._ankle._motor->on_off();
            exo_data.right_leg.ankle.motor.kp = 0;
            exo_data.right_leg.ankle.motor.kd = 0;
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
            
        }
        #ifdef MAIN_DEBUG
          Serial.println("Superloop :: Motor Charging Delay - Please be patient");
        #endif 
        exo_data.status = status_defs::messages::motor_start_up; 
        int motor_start_delay_ms = 60000;
        int motor_start_time = millis();
        int dot_print_ms = 1000;
        int last_dot_time = millis();
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
        // wait for all motors to enable
//        while ((exo_data.left_leg.hip.is_used ? exo.left_leg._hip._motor->enable(exo.left_leg._hip._motor->_motor_data->enabled): 1) 
//                && (exo_data.right_leg.hip.is_used ? exo.right_leg._hip._motor->enable(exo.right_leg._hip._motor->_motor_data->enabled) : 1)
//                && (exo_data.left_leg.ankle.is_used ? exo.left_leg._ankle._motor->enable(exo.left_leg._ankle._motor->_motor_data->enabled): 1) 
//                && (exo_data.right_leg.ankle.is_used ? exo.right_leg._ankle._motor->enable(exo.right_leg._ankle._motor->_motor_data->enabled) : 1) )
//        {
//          
//        }
        #ifdef MAIN_DEBUG
            Serial.println("Superloop :: Motors Enabled");
            Serial.println("Superloop :: Parameters Set");
            Serial.println("Superloop :: End First Run Conditional");
        #endif
    }

    static bool static_calibration_done = false;
    unsigned int pause_after_static_calibration_ms = 10000;
    static unsigned int time_dynamic_calibration_finished; 
    static bool pause_between_calibration_done = false;   
    static bool dynamic_calibration_done = false;

    
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

    
    
    if ((!static_calibration_done) && (!exo_data.left_leg.ankle.calibrate_torque_sensor && !exo_data.right_leg.ankle.calibrate_torque_sensor))
    {
        #ifdef MAIN_DEBUG
          Serial.println("Superloop : Static Calibration Done");
        #endif
        static_calibration_done  = true;
        time_dynamic_calibration_finished = millis();
        exo_data.status = status_defs::messages::test;
    }

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

    comms.handle_ble();
    comms.local_sample();
    comms.update_gui();

    exo.run();
    int dot_print_ms = 5000;
    static int last_dot_time = millis();
    if(millis() - last_dot_time > dot_print_ms)
    {
      last_dot_time = millis();
      Serial.print(".");
    }
}


#elif defined(ARDUINO_ARDUINO_NANO33BLE)  // board name is ARDUINO_[build.board] property in the board.txt file here found at C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\hardware\mbed_nano\2.6.1  They just already prepended it with ARDUINO so you have to do it twice.
#include <stdint.h>
#include "src/ParseIni.h"
#include "src/ExoData.h"
#include "src/ComsMCU.h"

//namespace config_info
//{
//    uint8_t (config_to_send)[ini_config::number_of_keys];
//}

namespace config_info
{
    uint8_t (config_to_send)[ini_config::number_of_keys] = {
      1,
      2,
      3,
      1,
      2,
      1,
      3,
      1,
      1,
      1,
      1,
      1,
      1,
    };
}

void setup()
{
    // TODO: ask for init data over spi
    
    Serial.begin(115200);
    while (!Serial);
}

void loop()
{
    static ExoData* exo_data = new ExoData(config_info::config_to_send);
    static ComsMCU* mcu = new ComsMCU(exo_data, config_info::config_to_send);
    mcu->handle_ble();
    mcu->local_sample();
    // TODO: SPI Transaction
    
    mcu->update_gui();
}

#else
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
