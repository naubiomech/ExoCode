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
  #include "src\FSR.h"
  
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

    Serial.print("heel");
    Serial.print("\t");
    Serial.print("toe");
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
    bool is_left = false;
    static FSR heel_fsr(is_left ? logic_micro_pins::fsr_sense_left_heel_pin : logic_micro_pins::fsr_sense_right_heel_pin); // Check if it is the left and use the appropriate pin for the side.
    static FSR toe_fsr(is_left ? logic_micro_pins::fsr_sense_left_toe_pin : logic_micro_pins::fsr_sense_right_toe_pin);
        
    int state_period_ms = 1;
    static int last_transition_time = millis();
    int current_time = millis();
    


    static bool do_calibration_toe_fsr = true; //bit 0 is calibrate fsr, bit 1 is refine calibration.
    static bool do_calibration_refinement_toe_fsr = true; 
    static bool do_calibration_heel_fsr = true; //bit 0 is calibrate fsr, bit 1 is refine calibration.
    static bool do_calibration_refinement_heel_fsr = true;
    
    if (state_period_ms <= (current_time - last_transition_time))
    {

      if (do_calibration_toe_fsr)
        {
            do_calibration_toe_fsr = toe_fsr.calibrate(do_calibration_toe_fsr);
        }
        else if (do_calibration_refinement_toe_fsr) 
        {
            do_calibration_refinement_toe_fsr = toe_fsr.refine_calibration(do_calibration_refinement_toe_fsr);
        }
        
        if (do_calibration_heel_fsr)
        {
            do_calibration_heel_fsr = heel_fsr.calibrate(do_calibration_heel_fsr);
        }
        else if (do_calibration_refinement_heel_fsr) 
        {
            do_calibration_refinement_heel_fsr = heel_fsr.refine_calibration(do_calibration_refinement_heel_fsr);
        }

      float heel_val = heel_fsr.read();
      float toe_val = toe_fsr.read();
      
      last_transition_time = current_time;
      Serial.print(heel_val);
      Serial.print("\t");
      Serial.print(toe_val);
      Serial.print("\n");
      
    }
    
  }


#endif
