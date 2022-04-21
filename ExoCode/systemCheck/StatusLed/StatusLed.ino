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
  #include "src\StatusLed.h"
  
  namespace config_info
  {
    uint8_t (config_to_send)[ini_config::number_of_keys];
  }
  
  void setup()
  {
    Serial.begin(115200);
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
    static StatusLed status_led(logic_micro_pins::status_led_r_pin, logic_micro_pins::status_led_g_pin, logic_micro_pins::status_led_b_pin);
  
    uint8_t num_messages = 5;
    static int current_message = 0;
    
    int state_period_ms = 10000;
    static int last_transition_time = millis();
    int current_time = millis();

    
    if (state_period_ms <= (current_time - last_transition_time))
    {
      current_message++;
      if (current_message >= num_messages)
      {
        current_message = 0;
        Serial.println("reset");
      }
      
      Serial.println(current_message);
      last_transition_time = current_time;
    }
    status_led.update(current_message);
  }


#endif
