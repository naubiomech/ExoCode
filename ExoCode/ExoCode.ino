/*
   Code used to run the exo from the teensy.  This communicates with the nano over SPI.

   P. Stegall Jan 2022
*/

#if defined(ARDUINO_TEENSY36)

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
#include "src\TSPISlave.h"


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

void setup()
{

  Serial.begin(115200);
  //TODO: Remove serial while for deployed version as this would hang
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  //        Serial.println("Teensy Microcontroller");


  ini_parser(config_info::config_to_send);
  //uint8_t (config_to_send)[ini_config::number_of_keys];

  //        Serial.println();
  //        Serial.println("setup : Coded Config Data");
  //        for (int i = 0; i<ini_config::number_of_keys; i++)
  //        {
  //            Serial.print("[");
  //            Serial.print(i);
  //            Serial.print("] : ");
  //            Serial.println((int)config_info::config_to_send[i]);
  //
  //        }
  //        Serial.println();
  //
  //        Serial.println();
  //        Serial.println(static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_info::config_to_send[config_defs::exo_side_idx]);

  // Now that we have read the config file create the data structure and exoskeleton object.

}

//
void loop()
{
  static ExoData exo_data(config_info::config_to_send);
  static Exo exo(&exo_data);




  //led::sync_led.update_led();  // actually change the led state, this also updates ledIsOn for recording the actual on/off state

  // Need to update this for the new bluetooth stream
  //        if (led::sync_led.do_blink | led::sync_led.do_start_stop_sequence) //if we are within a trial stream data
  //        {
  //          //stream = 1;
  //        }
  //        else
  //        {
  //          //stream = 0;
  //        }

  /* Temp code to test the FSR */
  //+++++++++++++++++++++++++++++++++++++++++
  /*          static bool first_run = true;
            static unsigned int ground_strike_count = 0;

            if (first_run)
            {
              exo_data.left_leg.do_calibration_toe_fsr = true;
              exo_data.left_leg.do_calibration_refinement_toe_fsr = true;

              exo_data.left_leg.do_calibration_heel_fsr = true;
              exo_data.left_leg.do_calibration_refinement_heel_fsr = true;

              first_run = false;
            }

            exo.left_leg.check_calibration();
            exo.left_leg.read_data();

            if(!exo_data.left_leg.do_calibration_toe_fsr & !exo_data.left_leg.do_calibration_refinement_toe_fsr & !exo_data.left_leg.do_calibration_heel_fsr & !exo_data.left_leg.do_calibration_refinement_heel_fsr)
            {
    //              Serial.print("toe reading : \t");
    //              Serial.print(exo_data.left_leg.toe_fsr);
    //              Serial.print("\t heel reading : \t");
    //              Serial.println(exo_data.left_leg.heel_fsr);
                if (exo_data.left_leg.ground_strike)
                {
                    ground_strike_count++;
    //                  Serial.print("Main - ground_strike_count = ");
    //                  Serial.println(ground_strike_count);
    //
    //                  Serial.println("");
                }

            }
    //          int print_time_ms = 100;
    //          static int last_timestamp = millis();
    //          int timestamp = millis();
    //          if ((timestamp-last_timestamp)>print_time_ms)
    //          {
    //              Serial.print("Main - percent_gait_x10 = ");
    //              Serial.print(exo_data.left_leg.percent_gait_x10);
    //              Serial.print("\tToe FSR = ");
    //              Serial.print(exo_data.left_leg.toe_fsr);
    //              Serial.print("\r");
    //              last_timestamp = timestamp;
    //          }
  */
  //+++++++++++++++++++++++++++++++++++++++++

  /*Temp code to test controllers*/
  //-----------------------------------------------
  //          //if(!exo_data.left_leg.do_calibration_toe_fsr & !exo_data.left_leg.do_calibration_refinement_toe_fsr & !exo_data.left_leg.do_calibration_heel_fsr & !exo_data.left_leg.do_calibration_refinement_heel_fsr)
  //          //{
  //              exo_data.left_leg.ankle.controller.parameters[controller_defs::proportional_joint_moment::max_torque_idx] = 2000;
  //              exo_data.left_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::pjmc;
  //
  //              exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::flexion_setpoint_idx] = 100;
  //              exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::extension_setpoint_idx] = -200;
  //              exo_data.left_leg.hip.controller.parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] = 80;
  //              exo_data.left_leg.hip.controller.controller = (uint8_t)config_defs::hip_controllers::extension_angle;
  //
  //              exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);
  //              exo.left_leg._hip.set_controller(exo_data.left_leg.hip.controller.controller);
  //
  //              // create a false angle and velocity signal for the hip.
  //              int hip_period = 5000000; //us
  //              static long last_loop_timestamp = micros();
  //              long loop_timestamp = micros();
  //              float last_p = exo_data.left_leg.hip.motor.p;
  //              exo_data.left_leg.hip.motor.p = utils::degrees_to_radians(30) + utils::degrees_to_radians(60) * sin(2 * PI * loop_timestamp / hip_period);
  //              exo_data.left_leg.hip.motor.v = utils::degrees_to_radians(60) * (2*PI/hip_period) * cos(2 * PI * loop_timestamp / hip_period); //(exo_data.left_leg.hip.motor.p - last_p)/(loop_timestamp - last_loop_timestamp);
  //              last_loop_timestamp = loop_timestamp;
  //
  //              exo.left_leg._hip.run_joint();
  //              exo.left_leg._ankle.run_joint();
  //
  //              int print_time_ms = 100;
  //              static int last_timestamp = millis();
  //              int timestamp = millis();
  //              if ((timestamp-last_timestamp)>print_time_ms)
  //              {
  ////                  Serial.print("Main -\n\ttorque CMD = ");
  ////                  Serial.println(exo_data.left_leg.ankle.controller.setpoint);
  ////                  Serial.print("\tToe FSR = ");
  ////                  Serial.println(exo_data.left_leg.toe_fsr);
  //                  //Serial.print("\t\t\t\t\t\t\r");
  //
  ////                  Serial.print(exo_data.left_leg.ankle.controller.setpoint);
  ////                  Serial.print(" ");
  //
  //                  Serial.print(utils::radians_to_degrees(exo_data.left_leg.hip.motor.v) * 1000000);
  //                  Serial.print(" ");
  //                  Serial.print(exo_data.left_leg.hip.controller.setpoint);
  //                  Serial.print(" ");
  //                  Serial.print(utils::radians_to_degrees(exo_data.left_leg.hip.motor.p) * 1);
  //                  Serial.println(" ");
  //
  //                  last_timestamp = timestamp;
  //              }
  //
  //          //}

  //-----------------------------------------------

  //          exo_data.left_leg.ankle.controller.parameters[controller_defs::zhang_collins::mass_idx] = 100;
  //          exo_data.left_leg.ankle.controller.parameters[controller_defs::zhang_collins::peak_normalized_torque_mNm_idx] = 200;  // should give 20 Nm for a 100 kg person
  //          // these are average values from the zhang collins paper
  //          exo_data.left_leg.ankle.controller.parameters[controller_defs::zhang_collins::t0_x10_idx] = 0;
  //          exo_data.left_leg.ankle.controller.parameters[controller_defs::zhang_collins::t1_x10_idx] = 271;
  //          exo_data.left_leg.ankle.controller.parameters[controller_defs::zhang_collins::t2_x10_idx] = 504;
  //          exo_data.left_leg.ankle.controller.parameters[controller_defs::zhang_collins::t3_x10_idx] = 627;
  //
  //          exo_data.left_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zhang_collins;
  //
  //          exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);
  //
  //          // create a false percent gait for the controller to use.
  //          int percent_gait_period = 5000000; //us
  //          static int step_start_timestamp = micros();
  //          int current_timestamp = micros();
  //          exo_data.left_leg.percent_gait_x10 = 1000 *  (float)(current_timestamp - step_start_timestamp) / percent_gait_period;
  //          if (exo_data.left_leg.percent_gait_x10 > 1000)
  //          {
  //              exo_data.left_leg.percent_gait_x10 = 0;
  //              step_start_timestamp = current_timestamp;
  //          }
  //
  //          exo.left_leg._ankle.run_joint();
  //
  //          int print_time_ms = 100;
  //          static int last_print_timestamp = millis();
  //          int print_timestamp = millis();
  //          if ((print_timestamp-last_print_timestamp)>print_time_ms)
  //          {
  //              Serial.print(exo_data.left_leg.ankle.controller.setpoint);
  //              Serial.print(" ");
  //              Serial.print(exo_data.left_leg.percent_gait_x10 * 10);
  //              Serial.println(" ");
  //              last_print_timestamp = print_timestamp;
  //          }

  //-----------------------------------------------

  /*
     Temp code to test torque sensor
  */
  static bool first_run = true;

  if (first_run)
  {
    exo_data.left_leg.hip.calibrate_torque_sensor = true;
    exo_data.left_leg.ankle.calibrate_torque_sensor = true;
    Serial.print("HipTorqueReading");
    Serial.print(" ");
    Serial.println("AnkleTorqueReading");

    first_run = false;
  }

  //        exo.left_leg.check_calibration();
  //        exo.left_leg.read_data();

  exo.run();

  //        int print_time_ms = 100;
  //        static int last_print_timestamp = millis();
  //        int print_timestamp = millis();
  //        if ((print_timestamp-last_print_timestamp)>print_time_ms)
  //        {
  //            Serial.print(exo_data.left_leg.hip.torque_reading);
  //            Serial.print(" ");
  //            Serial.print(exo_data.left_leg.ankle.torque_reading);
  //            Serial.println(" ");

  //            Serial.print(exo_data.left_leg.hip.calibrate_torque_sensor);
  //            Serial.print(" ");
  //            Serial.print(exo_data.left_leg.ankle.calibrate_torque_sensor);
  //            Serial.println(" ");
  //            last_print_timestamp = print_timestamp;

  //        }


  //-----------------------------------------------


  /*
     Test Sync LED
  */
  int sync_trigger_timestamp_ms = millis();
  static int last_sync_trigger_timestamp_ms = sync_trigger_timestamp_ms;
  const int trigger_period_ms = 5000;
  if ((sync_trigger_timestamp_ms - last_sync_trigger_timestamp_ms) >= trigger_period_ms)
  {
    exo.sync_led.trigger();
    Serial.println("Sync LED Triggered");
    last_sync_trigger_timestamp_ms = sync_trigger_timestamp_ms;
  }

  int print_time_ms = 100;
  int print_timestamp = millis();
  static int last_print_timestamp = print_timestamp;

  if ((print_timestamp - last_print_timestamp) >= print_time_ms)
  {
    //Serial.println(exo_data.sync_led_state);
  }


  /*
     Test Status LED
  */
  int status_trigger_timestamp_ms = millis();
  static int last_status_trigger_timestamp_ms = status_trigger_timestamp_ms;
  const int status_period_ms = 1000;
  if ((status_trigger_timestamp_ms - last_status_trigger_timestamp_ms) >= status_period_ms)
  {
      exo_data.status++;
      if (exo_data.status > 3)
      {
          exo_data.status = 0;
      }
      switch (exo_data.status)
      {
          case 0 :
            Serial.println("Status: off");
            break;
          case 1 :
            Serial.println("Status: trial off");
            break;
          case 2 :
            Serial.println("Status: trial on");
            break;
          case 3 :
            Serial.println("Status: error");
            break;
          default :
            Serial.println("Status: not defined");
            break;  
      }
      last_status_trigger_timestamp_ms = status_trigger_timestamp_ms;
  }

}

#elif defined(ARDUINO_ARDUINO_NANO33BLE)  // board name is ARDUINO_[build.board] property in the board.txt file here found at C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\hardware\mbed_nano\2.6.1  They just already prepended it with ARDUINO so you have to do it twice.
#include "src\ParseIni.h"
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
  for (int i = 0; i < ini_config::number_of_keys; i++)
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
