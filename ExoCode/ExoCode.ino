/*
   Code used to run the exo from the teensy.  This communicates with the nano over SPI.

   P. Stegall Jan 2022
*/
#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)

#define INCLUDE_FLEXCAN_DEBUG

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
#if defined(ARDUINO_TEENSY36)
  #include <TSPISlave.h>
#elif defined(ARDUINO_TEENSY41)
  #include "SPISlave_T4.h"
#endif
//#include "src\Motor.h"

/*namespace led{
    IntervalTimer sync_timer;  // Create a timer for setting for handling the interupt, this is needed as the ISR cannot access class variables since you cannot pass a self to the ISR.
    SyncLed sync_led(logic_micro_pins::sync_led_pin, SYNC_START_STOP_HALF_PERIOD_US, SYNC_HALF_PERIOD_US, logic_micro_pins::sync_led_on_state, logic_micro_pins::sync_default_pin);  // Create a sync LED object, the first and last arguments (pin) are found in Board.h, and the rest are in Sync_Led.h.  If you do not have a digital input for the default state you can remove SYNC_DEFAULT_STATE_PIN.
    StatusLed status_led(logic_micro_pins::status_led_r_pin, logic_micro_pins::status_led_g_pin, logic_micro_pins::status_led_b_pin);  // Create the status LED object.

  }*/
// this creates a does not name a type error not sure why as it worked before.  Could be an issue with the lib.  Commenting out for now.
//led::sync_timer.begin(Led::grossLedInteruptWrapper, Led::sync_led.currentSyncPeriod);




namespace config_info
{
uint8_t (config_to_send)[ini_config::number_of_keys];

}


void callback()//executed every 2ms
{


}

//inline float modulate_torque(float& count)
//{
//    const float pi = 3.14159;
//    count = (count >= 10*pi) ? 0:count;
//    count += pi/2000;
//    return sin(count/pi);
//}

void setup()
{
  Serial.begin(115200);
//  TODO: Remove serial while for deployed version as this would hang
  // while (!Serial) {
    // ; // wait for serial port to connect. Needed for native USB
  // }

  ini_parser(config_info::config_to_send);
  
  
//  for (int i = 0; i < ini_config::number_of_keys; i++)
//  {
//    Serial.print("[");
//    Serial.print(i);
//    Serial.print("] : ");
//    Serial.print((int)config_info::config_to_send[i]);
//    Serial.print("\n");
//  }
//    Serial.print("\n");
//    delay(60000);
    
  // Now that we have read the config file create the data structure and exoskeleton object.

  
}



void loop()
{
    static ExoData exo_data(config_info::config_to_send);
    static Exo exo(&exo_data);
    
//    static AK80 left_ankle = AK80(config_defs::joint_id::left_ankle, &exo_data);
//    static AK80 right_ankle = AK80(config_defs::joint_id::right_ankle, &exo_data);
//    static AK60 left_hip = AK60(config_defs::joint_id::left_hip, &exo_data);
//    static AK60 right_hip = AK60(config_defs::joint_id::right_hip, &exo_data);

//    static int cnt = 0;
//    Serial.print("Superloop :: cnt = ");
//    Serial.print(cnt++);
//    Serial.print("\n\r");    

    
    static bool first_run = true;
    if (first_run)
    {
        first_run = false;
//        Serial.print("Superloop :: Start First Run Conditional");
        // Need to comment out motors not in use otherwise code hangs.
        exo.left_leg._hip._motor->_motor_data->enabled = true;
        exo.right_leg._hip._motor->_motor_data->enabled = true;

        exo.left_leg._hip._motor->on_off(exo.left_leg._hip._motor->_motor_data->enabled);
        exo.right_leg._hip._motor->on_off(exo.right_leg._hip._motor->_motor_data->enabled);
        
        // This is hacky as the first one doesn't go through.  
        // Changing initalization order of legs in Exo changes behaviour.  
        // First initialized fails to enable, independent of what is here.
        // TODO : Figure out why the first initalization fails.
//        exo.left_leg._hip._motor->on_off(false);
//        exo.left_leg._hip._motor->on_off(true);  
        Serial.print("\nSuperloop : Line 117\n");
        
//        Serial.print("Superloop :: Motors Turned on/off");

//        exo_data.left_leg.ankle.motor.kp = 0;
//        exo_data.left_leg.ankle.motor.kd = 0;

        exo_data.left_leg.hip.motor.kp = 0;
        exo_data.left_leg.hip.motor.kd = 0;

        exo_data.right_leg.hip.motor.kp = 0;
        exo_data.right_leg.hip.motor.kd = 0;
//        Serial.print("Superloop :: Motor gains set");

        // TODO : Confirm that motors are zeroing
        // Left zeros, right doesn't.
        // Possibly related to enabling issue.
        // Adding another zeroing for the right leg doesn't help.  
        Serial.print("\nSuperloop : Line 130\n");
        exo.left_leg._hip._motor->zero();
        Serial.print("\nSuperloop : Line 132\n");
        exo.right_leg._hip._motor->zero();
        Serial.print("\nSuperloop : Line 134\n");
//        exo.right_leg._hip._motor->zero();
//        Serial.print("\nSuperloop : Line 134\n");
//        Serial.print("Superloop :: Motors zeroed");

// Left leg controller selection, comment out the ones not used.
// Extension Angle *******************************        
//        exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::flexion_setpoint_idx] = 2;
//        exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::extension_setpoint_idx] = -8; // this should be negative
//        exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] = 80;
//        exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::angle_threshold_idx] = utils::degrees_to_radians(5);
//        exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
//        exo_data.left_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::extension_angle);
// Bang Bang *******************************
        exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::flexion_setpoint_idx] = 2;
        exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::extension_setpoint_idx] = -4; // this should be negative
        exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::target_flexion_percent_max_idx] = 80;
        exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::angle_threshold_idx] = utils::degrees_to_radians(5);
        exo_data.left_leg.hip.controller.parameters[controller_defs::bang_bang::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
        exo_data.left_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::bang_bang);
// Zero Torque *******************************        
//        exo_data.left_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::zero_torque);
//===============================
        exo.left_leg._hip.set_controller(exo_data.left_leg.hip.controller.controller);


// Right leg controller selection, comment out the ones not used.        
// Extension Angle *******************************
//        exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::flexion_setpoint_idx] = 2;
//        exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::extension_setpoint_idx] = -8;  // this should be negative
//        exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] = 80;
//        exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::angle_threshold_idx] = utils::degrees_to_radians(5);
//        exo_data.right_leg.hip.controller.parameters[controller_defs::extension_angle::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
//        exo_data.right_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::extension_angle);
// Bang Bang *******************************
        exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::flexion_setpoint_idx] = 2;
        exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::extension_setpoint_idx] = -4;  // this should be negative
        exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::target_flexion_percent_max_idx] = 80;
        exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::angle_threshold_idx] = utils::degrees_to_radians(5);
        exo_data.right_leg.hip.controller.parameters[controller_defs::bang_bang::velocity_threshold_idx] = utils::degrees_to_radians(-10);// this should be negative
        exo_data.right_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::bang_bang);
// Zero Torque*******************************
//        exo_data.right_leg.hip.controller.controller = uint8_t(config_defs::hip_controllers::zero_torque);
//===============================
        exo.right_leg._hip.set_controller(exo_data.right_leg.hip.controller.controller);
//        Serial.print("Superloop :: Controller Set");
        
        
//        exo_data.left_leg.ankle.controller.parameters[controller_defs::proportional_joint_moment::max_torque_idx] = 2;
//        exo_data.left_leg.ankle.controller.controller = uint8_t(config_defs::ankle_controllers::pjmc);
//        exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);

//        exo_data.left_leg.do_calibration_toe_fsr = true;
//        exo_data.left_leg.do_calibration_refinement_toe_fsr = true;
//        exo_data.left_leg.do_calibration_heel_fsr = true;
//        exo_data.left_leg.do_calibration_refinement_heel_fsr = true;

//        bool do_calibration_toe_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
//        bool do_calibration_refinement_toe_fsr; 
//        bool do_calibration_heel_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
//        bool do_calibration_refinement_heel_fsr;

//        Serial.print("Superloop :: End First Run Conditional");

          Serial.print("Left_hip_angle, ");
          Serial.print("Right_hip_angle, ");
          Serial.print("Left_hip_setpoint, ");
          Serial.print("Right_hip_setpoint, ");
          Serial.print("\n");
    }

    
    static float old_time = micros();
    float new_time = micros();
    if(new_time - old_time > 50)
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
        Serial.print(exo_data.left_leg.hip.position);
        Serial.print(", ");
        Serial.print(exo_data.right_leg.hip.position);
        Serial.print(", ");
        Serial.print(exo_data.left_leg.hip.controller.setpoint);
        Serial.print(", ");
        Serial.print(exo_data.right_leg.hip.controller.setpoint);
        Serial.print("\n");

        
    }

    exo.run();

    
//    Serial.print("\n");
}


#elif defined(ARDUINO_ARDUINO_NANO33BLE)  // board name is ARDUINO_[build.board] property in the board.txt file here found at C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\hardware\mbed_nano\2.6.1  They just already prepended it with ARDUINO so you have to do it twice.
#include <stdint.h>
#include "src/ParseIni.h"
#include "src/ExoData.h"
#include "src/ComsMCU.h"

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
    Serial.println("Starting!");
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

