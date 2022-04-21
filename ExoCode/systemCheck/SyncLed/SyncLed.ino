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
  #include "src\SyncLed.h"
  
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
    static SyncLed sync_led(logic_micro_pins::sync_led_pin, SYNC_START_STOP_HALF_PERIOD_US, SYNC_HALF_PERIOD_US, logic_micro_pins::sync_led_on_state, logic_micro_pins::sync_default_pin);  // Create a sync LED object, the first and last arguments (pin) are found in Board.h, and the rest are in Sync_Led.h.  If you do not have a digital input for the default state you can remove SYNC_DEFAULT_STATE_PIN.  
     
    int state_period_ms = 10000;
    static int last_transition_time = millis();
    int current_time = millis();
    static int trigger_count = 0;
    
    if (state_period_ms <= (current_time - last_transition_time))
    {
      trigger_count++;
      Serial.println(trigger_count);
      sync_led.trigger();
      last_transition_time = current_time;
    }
    sync_led.handler();
  }


#endif
