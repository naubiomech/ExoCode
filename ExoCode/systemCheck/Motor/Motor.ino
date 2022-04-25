/*
   Code to test the Status LED
   P. Stegall April 2022
*/
#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)
  // general headers for using the system
  #include "src\Board.h"
  #include "src\Utilities.h"
  #include "src\ParseIni.h"
  
  // header for the component we are checking.
  #include "src\Motor.h"
  #include "src\ExoData.h"
  #include "src\Joint.h"
  #include <math.h>
  
  namespace config_info
  {
    uint8_t (config_to_send)[ini_config::number_of_keys];
  }
  
  void setup()
  {
    Serial.begin(115200);
    while(!Serial)
    {
      ;
    }
    // enable the estop pullup.
    pinMode(logic_micro_pins::motor_stop_pin,INPUT_PULLUP);

    Serial.print("Left_hip_angle, ");
    Serial.print("Right_hip_angle, ");
    Serial.print("Left_hip_setpoint, ");
    Serial.print("Right_hip_setpoint, ");
    Serial.print("\n");

    
//    #if BOARD_VERSION == AK_Board_V0_1
//      Serial.println("Board : AK_Board_V0_1");
//    #elif BOARD_VERSION == AK_Board_V0_3
//      Serial.println("Board : AK_Board_V0_3");
//    #endif
//  
//    #if defined(ARDUINO_TEENSY36)
//      Serial.println("Teensy 3.6");
//    #elif defined(ARDUINO_TEENSY41)
//      Serial.println("Teensy 4.1");
//    #endif
//  
//    
//    
//    Serial.print(logic_micro_pins::status_led_r_pin);
//    Serial.print("\t");
//    Serial.print(logic_micro_pins::status_led_g_pin);
//    Serial.print("\t");
//    Serial.print(logic_micro_pins::status_led_b_pin);
//    Serial.print("\n");
    
  }
  
  void loop()
  {
    config_info::config_to_send[config_defs::board_name_idx] = (uint8_t)config_defs::board_name::AK_board;
    config_info::config_to_send[config_defs::board_version_idx] = (uint8_t)config_defs::board_version::zero_three;
    config_info::config_to_send[config_defs::battery_idx] = (uint8_t)config_defs::battery::dumb;
    config_info::config_to_send[config_defs::exo_name_idx] = (uint8_t)config_defs::exo_name::bilateral_hip;
    config_info::config_to_send[config_defs::exo_side_idx] = (uint8_t)config_defs::exo_side::bilateral;
    config_info::config_to_send[config_defs::hip_idx] = (uint8_t)config_defs::motor::AK60;
    config_info::config_to_send[config_defs::knee_idx] = (uint8_t)config_defs::motor::not_used;
    config_info::config_to_send[config_defs::ankle_idx] = (uint8_t)config_defs::motor::not_used;
    config_info::config_to_send[config_defs::hip_gear_idx] = (uint8_t)config_defs::gearing::gearing_1_1;
    config_info::config_to_send[config_defs::knee_gear_idx] = (uint8_t)config_defs::gearing::gearing_1_1;
    config_info::config_to_send[config_defs::ankle_gear_idx] = (uint8_t)config_defs::gearing::gearing_1_1;
    config_info::config_to_send[config_defs::exo_hip_default_controller_idx] = (uint8_t)config_defs::hip_controllers::disabled;
    config_info::config_to_send[config_defs::exo_knee_default_controller_idx] = (uint8_t)config_defs::knee_controllers::disabled;
    config_info::config_to_send[config_defs::exo_ankle_default_controller_idx] = (uint8_t)config_defs::ankle_controllers::disabled;
    config_info::config_to_send[config_defs::hip_flip_dir_idx] = (uint8_t)config_defs::flip_dir::right;
    config_info::config_to_send[config_defs::knee_flip_dir_idx] = (uint8_t)config_defs::flip_dir::neither;
    config_info::config_to_send[config_defs::ankle_flip_dir_idx] = (uint8_t)config_defs::flip_dir::neither;
    
    static ExoData exo_data(config_info::config_to_send);
    
    
    
    // these should be changed to match the ID of the motors.
    static AK60 right_motor(config_defs::joint_id::right_hip, &exo_data, _Joint::get_motor_enable_pin(config_defs::joint_id::right_hip, &exo_data));
    static AK60 left_motor(config_defs::joint_id::left_hip, &exo_data, _Joint::get_motor_enable_pin(config_defs::joint_id::left_hip, &exo_data));
    
    

    static int motor_enable_time = millis();
    int time_to_stay_on_ms = 20000;
//    left_motor.on_off(true);
//    right_motor.on_off(true);
          
    int state_period_ms = 1;
    float left_magnitude = 0;//.5;
    float right_magnitude = 0;//1;
    static int last_transition_time = millis();
    int current_time = millis();
    
    
    static int pattern_start_timestamp = millis();
    int pattern_period_ms = 2000;
    
    static bool first_run = true;
    if (first_run)
    {
        first_run = false;
  
        left_motor._motor_data->enabled = true;
        right_motor._motor_data->enabled = true;

        left_motor.on_off(left_motor._motor_data->enabled);
        right_motor.on_off(right_motor._motor_data->enabled);
        
        right_motor.zero();
        left_motor.zero();
    }
    

    
    if (state_period_ms <= (current_time - last_transition_time))
    {
      int timestamp = millis();
//      Serial.print("Superloop : time since enable = ");
//      Serial.print(timestamp - motor_enable_time);
//      Serial.print("\n");

      if (time_to_stay_on_ms < (timestamp - motor_enable_time))
      {
          left_motor._motor_data->enabled = false;
          right_motor._motor_data->enabled = false;
      }

      left_motor.on_off(left_motor._motor_data->enabled);
      right_motor.on_off(right_motor._motor_data->enabled);
      
      // This isn't the actual angle this is an angle used to create a sinusodal torque
      float angle_deg = 360.0 * (timestamp - pattern_start_timestamp) / pattern_period_ms;
      float left_torque_command = left_magnitude * sin (angle_deg * PI / 180);
      float right_torque_command = right_magnitude * sin (angle_deg * PI / 180);
      
      left_motor.transaction(left_torque_command);
      right_motor.transaction(right_torque_command);
      
      
      last_transition_time = current_time;
      if (left_motor._motor_data->enabled)
      {
          Serial.print(left_motor._motor_data->p);
          Serial.print("\t");
          Serial.print(right_motor._motor_data->p);
          Serial.print("\t");
          Serial.print(left_torque_command);
          Serial.print("\t");
          Serial.print(right_torque_command);
         
          Serial.print("\n");
      }
      
    }
    
  }


#endif
