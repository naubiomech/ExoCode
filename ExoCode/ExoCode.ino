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

void callback()//executed every 2ms
{


}

void setup()
{
  Serial.begin(115200);
//  TODO: Remove serial while for deployed version as this would hang
  // while (!Serial) {
    // ; // wait for serial port to connect. Needed for native USB
  // }

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
//    delay(60000);
    
  // Now that we have read the config file create the data structure and exoskeleton object.

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

    // used to check loop speed but can be removed.
//    static utils::SpeedCheck speed_check(33);
//    speed_check.toggle();
    
    
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

//        
//        
        // Need to comment out motors not in use otherwise code hangs.
        if (exo_data.left_leg.hip.is_used)
        {
            exo.left_leg._hip._motor->_motor_data->enabled = true;
            exo.left_leg._hip._motor->on_off(exo.left_leg._hip._motor->_motor_data->enabled);
            exo_data.left_leg.hip.motor.kp = 0;
            exo_data.left_leg.hip.motor.kd = 0;
            exo.left_leg._hip._motor->zero();
            set_controller_params((uint8_t) exo_data.left_leg.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data);
//            //wait till calibration is done
            exo_data.left_leg.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque; // start in zero torque

//          Extension Angle *******************************        
//            exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::flexion_setpoint_idx] = 2;
//            exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::extension_setpoint_idx] = -8; // this should be negative
//            exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] = 80;
//            exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::angle_threshold_idx] = utils::degrees_to_radians(5);
//            exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
//            exo_data.left_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::extension_angle);
  
//           Bang Bang *******************************
//            exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::flexion_setpoint_idx] = 2;
//            exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::extension_setpoint_idx] = -4; // this should be negative
//            exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::target_flexion_percent_max_idx] = 80;
//            exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::angle_threshold_idx] = utils::degrees_to_radians(5);
//            exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
//            exo_data.left_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::bang_bang);
  
//           Sine *************************************
//            exo_data.left_leg.hip.controller.parameters[controller_defs::sine::amplitude_idx] = 1;  // amplitude Nm
//            exo_data.left_leg.hip.controller.parameters[controller_defs::sine::period_idx] = 1000; // period ms
//            exo_data.left_leg.hip.controller.parameters[controller_defs::sine::phase_shift_idx] = utils::degrees_to_radians(0);  // phase shift rad
//            exo_data.left_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::sine);
          
//           Zero Torque *******************************        
//            exo_data.left_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::zero_torque);
  //===============================
            exo.left_leg._hip.set_controller(exo_data.left_leg.hip.controller.controller);

        }
        
        if (exo_data.right_leg.hip.is_used)
        {
            exo.right_leg._hip._motor->_motor_data->enabled = true;
            exo.right_leg._hip._motor->on_off(exo.right_leg._hip._motor->_motor_data->enabled);
            exo_data.right_leg.hip.motor.kp = 0;
            exo_data.right_leg.hip.motor.kd = 0;
            exo.right_leg._hip._motor->zero();
            set_controller_params((uint8_t) exo_data.right_leg.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data);
//            //wait till calibration is done
            exo_data.right_leg.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque; // start in zero torque

//          Extension Angle *******************************
//            exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::flexion_setpoint_idx] = 2;
//            exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::extension_setpoint_idx] = -8;  // this should be negative
//            exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] = 80;
//            exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::angle_threshold_idx] = utils::degrees_to_radians(5);
//            exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
//            exo_data.right_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::extension_angle);

//          Bang Bang *******************************
//            exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::flexion_setpoint_idx] = 2;
//            exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::extension_setpoint_idx] = -4;  // this should be negative
//            exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::target_flexion_percent_max_idx] = 80;
//            exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::angle_threshold_idx] = utils::degrees_to_radians(5);
//            exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
//            exo_data.right_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::bang_bang);

//          Sine *************************************
//            exo_data.right_leg.hip.controller.parameters[controller_defs::sine::amplitude_idx] = 1;  // amplitude Nm
//            exo_data.right_leg.hip.controller.parameters[controller_defs::sine::period_idx] = 1000; // period ms
//            exo_data.right_leg.hip.controller.parameters[controller_defs::sine::phase_shift_idx] = utils::degrees_to_radians(90);  // phase shift rad
//            exo_data.right_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::sine);
                
//          Zero Torque*******************************
//            exo_data.right_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::zero_torque);
//===============================
            exo.right_leg._hip.set_controller(exo_data.right_leg.hip.controller.controller);

        }

        if (exo_data.left_leg.ankle.is_used)
        {
            #ifdef MAIN_DEBUG
              Serial.println("Superloop :: Left Ankle Used");
            #endif
            exo.left_leg._ankle._motor->_motor_data->enabled = true;
            exo.left_leg._ankle._motor->on_off(exo.left_leg._ankle._motor->_motor_data->enabled);
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
//            //wait till calibration is done
            exo_data.left_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque; // start in zero torque

            
//          Sine *************************************
//            exo_data.left_leg.ankle.controller.parameters[controller_defs::sine::amplitude_idx] = 3;  // amplitude Nm
//            exo_data.left_leg.ankle.controller.parameters[controller_defs::sine::period_idx] = 1000; // period ms
//            exo_data.left_leg.ankle.controller.parameters[controller_defs::sine::phase_shift_idx] = utils::degrees_to_radians(0);  // phase shift rad
//            exo_data.left_leg.ankle.controller.controller = uint8_t(config_defs::ankle_controllers::sine);
            
//          Zero Torque*******************************
//            exo_data.left_leg.ankle.controller.controller = uint8_t(config_defs::ankle_controllers::zero_torque);
//===============================
            exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);              
        }
        
        if (exo_data.right_leg.ankle.is_used)
        {
            exo.right_leg._ankle._motor->_motor_data->enabled = true;
            exo.right_leg._ankle._motor->on_off(exo.right_leg._ankle._motor->_motor_data->enabled);
            exo_data.right_leg.ankle.motor.kp = 0;
            exo_data.right_leg.ankle.motor.kd = 0;
            exo.right_leg._ankle._motor->zero();
            set_controller_params((uint8_t) exo_data.right_leg.ankle.id, config_info::config_to_send[config_defs::exo_ankle_default_controller_idx], 0, &exo_data);
//            //wait till calibration is done
            exo_data.right_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque; // start in zero torque

//          Sine *************************************
//            exo_data.right_leg.ankle.controller.parameters[controller_defs::sine::amplitude_idx] = 3;  // amplitude Nm
//            exo_data.right_leg.ankle.controller.parameters[controller_defs::sine::period_idx] = 1000; // period ms
//            exo_data.right_leg.ankle.controller.parameters[controller_defs::sine::phase_shift_idx] = utils::degrees_to_radians(90);  // phase shift rad
//            exo_data.right_leg.ankle.controller.controller = uint8_t(config_defs::ankle_controllers::sine);
        
//          Zero Torque*******************************
//            exo_data.right_leg.ankle.controller.controller = uint8_t(config_defs::ankle_controllers::zero_torque);
//===============================
            exo.right_leg._ankle.set_controller(exo_data.right_leg.ankle.controller.controller);
            
        }
        
//        exo_data.left_leg.ankle.controller.parameters[controller_defs::proportional_joint_moment::max_torque_idx] = 2;
//        exo_data.left_leg.ankle.controller.controller = uint8_t(config_defs::ankle_controllers::pjmc);
//        exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);

        if(exo_data.left_leg.is_used)
        {
            exo_data.left_leg.ankle.calibrate_torque_sensor = true;
        }
       
        if(exo_data.right_leg.is_used)
        {
            exo_data.right_leg.ankle.calibrate_torque_sensor = true;  
        }
        

//        bool do_calibration_toe_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
//        bool do_calibration_refinement_toe_fsr; 
//        bool do_calibration_heel_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
//        bool do_calibration_refinement_heel_fsr;

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
//        exo.left_leg._ankle._motor->transaction();
//        exo.right_leg._ankle._motor->transaction();
//        exo.left_leg._hip._motor->transaction();
//        exo.right_leg._hip._motor->transaction();
//        send_count += 1;
//        if (send_count >= 100) {
//          Serial.print(exo_data.left_leg.ankle.motor.p);
//          Serial.print("\t");
//          Serial.print(exo_data.right_leg.ankle.motor.p);
//          Serial.print("\t");
//          Serial.print(exo_data.left_leg.hip.motor.p);
//          Serial.print("\t");
//          Serial.print(exo_data.right_leg.hip.motor.p);
//          Serial.print("\n");
//          send_count = 0;
//          }

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
        exo_data.status = status_led_defs::messages::test;
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

    exo.run();

    
//    Serial.print("\n");
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
    static ComsMCU* mcu = new ComsMCU(exo_data);
    mcu->handle_ble();
    mcu->local_sample();
    // TODO: Get New Data over SPI
    
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
