
#include "src\SPIHandler.h"
#include "src\Utilities.h"
#define SIMPLE_EXAMPLE 1 

#if defined(ARDUINO_TEENSY36)
    
    // I don't like having this here but I was having an issue with the spi object and myFunc having the right scope.
    namespace spi_peripheral
    {
        TSPISlave mySPI = TSPISlave(SPI, logic_micro_pins::miso_pin, logic_micro_pins::mosi_pin, logic_micro_pins::sck_pin, logic_micro_pins::cs_pin, logic_micro_pins::spi_mode);
        
        uint8_t spi_buffer[spi_cmd::max_data_len];
        
        void myFunc() 
        {
            spi_msg(mySPI, spi_cmd::max_data_len, config, &spi_buffer);
        }
    }

  void setup()
  {
    Serial.begin(115200);
    
    spi_peripheral::mySPI.onReceive(spi_peripheral::myFunc);
    
//    void myFunc() 
//    {
//        spi_msg(mySPI, spi_cmd::max_msg_len, spi_buffer);
//    }
  }
  
  void loop()
  {
  
    
  }

#elif defined(ARDUINO_TEENSY41)
    #include "SPISlave_T4.h"
    namespace config_info
    {
        uint8_t config_to_send[ini_config::number_of_keys] = {
            1,  // board name
            3,  // board version
            2,  // battery
            1,  // exo name
            1,  // exo side
            2,  // hip
            1,  // knee
            3,  // ankle
            1,  // hip gear
            1,  // knee gear
            1,  // ankle gear
            1,  // hip default controller
            1,  // knee default controller
            1,  // ankle default controller
            3,  // hip flip dir
            3,  // knee flip dir
            3,  // ankle flip dir
          };
    
    }


    #ifdef SIMPLE_EXAMPLE  
        // namespace to store variables used in t ge SPI callback
        namespace spi_peripheral
        {
            SPISlave_T4<&SPI, SPI_8_BITS> my_spi;
            const uint8_t message_len = 4;// length of the actual message
            uint8_t message_number = 0;// counter to keep track of changes
            bool loop_running = false;// not used intended to be used to signal the controller board that the peripheral is ready
            uint8_t controller_message[message_len+1]= {0,0,0,0,0};// end messages with extra zero that will be read by the other board at the start of the message
            uint8_t peripheral_message[message_len+1]= {0x04, 0xAA, 0xF0, 0x0F, 0x00};//message_length, message_num, data1, data2, dummy_val(must be 0)
            uint8_t debug_location;// used to see where we get in the callback
            bool is_unread_message = false;  // let the main loop know something new has come in
            void spi_callback() 
            {    
                uint8_t i = 0;// send idx
                uint8_t j = 0;// recv idx
                while ( my_spi.active() ) 
                {
                    if (my_spi.available()) 
                    {
                        if(0==i)
                        {
                          peripheral_message[1] = message_number;// pack current message num
                          Serial.print("spi_callback :: message_number : 0x");
                          Serial.println(message_number++,HEX); // print and increment message num
                        }
                        if ( j > sizeof(peripheral_message) ) // recieving data beyond message size likely due to error
                        {
                            //my_spi.pushr(0);
                        }
                        else// Still normally operating keep putting in data
                        {
                            my_spi.pushr(utils::ff_to_fe(peripheral_message[i]));//converts any FF to FE before sending as the message appears as FF on the other side when there is an error
                        }
                        
                        controller_message[j] = my_spi.popr();//pull the message off the buffer
                        i++;
                        
                        if((0xFF!=controller_message[j]&&0!=j)||(0==j && 0!=controller_message[j]))//remove leading 0s and ignore FF as the other side sends this back if it was received
                        {
                            j++;
                        }
                        is_unread_message = true;
                    }
                }
            }
        }

        void setup()
        {
            Serial.begin(115200);
            while (!Serial);
            Serial.println("\n\nSetup : Serial Started");
            spi_peripheral::my_spi.begin();
            Serial.println("Setup :: SPI Begin");
            spi_peripheral::my_spi.onReceive(spi_peripheral::spi_callback);
            Serial.println("Setup :: SPI callback set");
            //spi_peripheral::my_spi.popr();// thought this might be needed as the controller does
        }

        void loop()
        {
            static int time_of_last_message = millis();
           
            static bool first_run = true;
      
            if (first_run)
            {
                Serial.println("Superloop :: First Run Start");
                
                first_run = false;
            }
      
            if (spi_peripheral::is_unread_message)
            {
                spi_peripheral::loop_running = true;
                Serial.print("\n\n\n\n");  
                Serial.println("Superloop :: controller_message");
                print_message(spi_peripheral::controller_message, spi_peripheral::message_len+1);
                Serial.println("Superloop :: peripheral_message");
                print_message(spi_peripheral::peripheral_message, spi_peripheral::message_len+1);
                spi_peripheral::is_unread_message = false;
                time_of_last_message = millis();
            }
      
            if((millis() - time_of_last_message) > 1000)
            {
                Serial.println("+++++++++++++++++++++++++++++");
                time_of_last_message = millis();
            }
            
        }

    #else
        // I don't like having this here but I was having an issue with the spi object and functions in the callback having the right scope.
        namespace spi_peripheral
        {
    
            
            SPISlave_T4<&SPI, SPI_8_BITS> my_spi;
            const uint8_t message_len = 4;
            uint8_t controller_message[message_len]= {message_len,3,2,1};
            uint8_t peripheral_message[message_len];
            uint8_t debug_location;
            bool is_unread_message = false;
    
    
    
    //        uint8_t controller_message[spi_cmd::max_param_len];
    //        uint8_t cmd = spi_cmd::send_config::id;
    //        bool is_unread_message = false;
    //        uint16_t debug_location;
    
            
            
    //        ExoData* data; // !! THIS NEEDS TO GET POINTED TO THE RIGHT SPOT IN THE FIRST RUN OF THE LOOP AFTER THE ExoData INSTANCE IS CREATED.
            // !! Will also need to do a static_spi_handler::send_length(my_spi, get_data_len(config_info::config_to_send)); before attaching the callback to the SPI
            
    //        void (function_pointer_spi_handler) (SPISlave_T4<&SPI, SPI_8_BITS> my_spi, uint8_t* config_to_send, ExoData* data, uint8_t* cmd) = &(StaticSPIHandler::transaction);  //pointer to the function that will be used during the callback.  This will be filled in after we create the SPIHandler object.
            void spi_callback() 
            {
    //            (*function_pointer_spi_handler)(my_spi, config_info::config_to_send, data, );
    //            debug_location = static_spi_handler::peripheral_transaction(my_spi, config_info::config_to_send, data, &cmd, controller_message); 
    //            static_spi_handler::set_message_flag(&is_unread_message);// this really didn't need to be a function
    
                debug_location = simple_static_spi_handler::peripheral_transaction(my_spi, controller_message, peripheral_message, message_len);
    // read
    //            uint8_t i = 0;
    //            uint8_t temp_len = 0;
    //            uint8_t trash;
    //            uint8_t prev_val;
    //            uint8_t cur_val;
    //            while (my_spi.active()) 
    //            {
    //                if (my_spi.available()) 
    //                {
    //                      if (i++ > len) 
    //                      {
    //                        i=0;
    //                      }
    //                      
    ////                      controller_message[i] = i;
    //                
    ////                    if (0 == i)
    ////                    {
    ////                        temp_len = my_spi.popr();
    ////                        //i++;
    ////                    }
    ////                    else if (i <= len)
    ////                    {                    
    ////                        controller_message[i-1] = my_spi.popr();
    ////                        //i++;                        
    ////                    }
    ////                    else
    ////                    {
    ////                        trash = my_spi.popr();
    ////                        Serial.print(trash);
    ////                    }
    //                    
    //                    // controller_message[i] = i;
    //                    // Serial.print("static_spi_handler::read_message: controller_message[i] = 0x");
    //                    // Serial.println(controller_message[i-1],HEX);
    //                }
    //                // i++;
    //            }
    //            Serial.println();
    //            debug_location = 0;
    //            debug_location = utils::update_bit(debug_location, temp_len == len, simple_static_spi_handler::debug_lengths_match_bit);
    //// send
    //            my_spi.pushr(len);
    //            for(int i = 0; i<len; i++)
    //            {
    //                my_spi.pushr(peripheral_message[i]);
    //            }
    //            
    //            is_unread_message = true;
    //            return;
            }
        }
    
        void dummy()
        {
          return;
        }
    
      
    
      void setup()
      {
        Serial.begin(115200);
    //    while(!Serial);
        //delay(1000);
        Serial.println("\n\nSetup : Serial Started");
    //    Serial.print("\tcontroller_message address : 0x");
    //    Serial.println((long)&(spi_peripheral::controller_message[0]),HEX);
    //    Serial.print("\tcmd address : 0x");
    //    Serial.println((long)&spi_peripheral::cmd,HEX);
        
        
    //    spi_peripheral::my_spi.onReceive(spi_peripheral::spi_callback);
    //    //spi_peripheral::mySPI.swapPins();
    //    spi_peripheral::my_spi.begin();
      }
    
      void loop()
      {
    //      static ExoData exo_data(config_info::config_to_send);
    //
    //      static uint8_t prev_cmd = 0;
          static int time_of_last_message = millis();
          
          //static SPISlave_T4<&SPI, SPI_8_BITS> my_spi;
    //      static SPIHandler spi_handler(spi_peripheral::my_spi, config_info::config_to_send, &exo_data);
    
          static uint8_t start_val = 0xAA;
    
    //      for(int i = 0; i<spi_peripheral::len; i++)
    //      {
    //          spi_peripheral::peripheral_message[spi_peripheral::len - 1 - i] = start_val + i;
    //      }
    //      
    //      start_val = start_val + 1;
          
          spi_peripheral::peripheral_message[0] = 0x0A;
          spi_peripheral::peripheral_message[1] = 0xA0;
          spi_peripheral::peripheral_message[2] = 0xAA;
          
          
          
          static bool first_run = true;
    
          if (first_run)
          {
            Serial.println("Superloop :: First Run Start");
    //        spi_peripheral::data = &exo_data;
    //        Serial.println("Superloop :: SPI Data pointer updated");
    //        Serial.print("\tdata address : 0x");
    //        Serial.println((long)spi_peripheral::data,HEX);
            
            // Set the message for the first SPI transaction
    //        static_spi_handler::send_length(spi_peripheral::my_spi, static_spi_handler::get_data_len(config_info::config_to_send));
    //        Serial.println("Superloop :: sent length");
    //        static_spi_handler::send_config(spi_peripheral::my_spi, config_info::config_to_send);
    //        Serial.println("Superloop :: sent config");
            
            //spi_peripheral::function_pointer_spi_handler = &spi_handler.transaction;
            
            //spi_peripheral::my_spi.onReceive(spi_peripheral::spi_callback);
            Serial.println("Superloop :: SPI callback set");
            
            //spi_peripheral::my_spi.begin();
            Serial.println("Superloop :: SPI Begin");
            first_run = false;
          }
    
          if (spi_peripheral::is_unread_message)
          {
              Serial.print("\n\n\n\n");
              
              //uint8_t debug_location = static_spi_handler::peripheral_transaction(spi_peripheral::my_spi, config_info::config_to_send, spi_peripheral::data, &(spi_peripheral::cmd), spi_peripheral::controller_message); 
              
              //Serial.print("Superloop:: spi_peripheral::cmd = ");
    
    //            static_spi_handler::read_message(spi_peripheral::my_spi, spi_peripheral::controller_message); 
    
    
    //            for(int i = 0 ; i<static_spi_handler::get_data_len(config_info::config_to_send) ; i++)
    //            {
    //                //static_spi_handler::send_length(spi_peripheral::my_spi, static_spi_handler::get_data_len(config_info::config_to_send));
    ////                Serial.println("Superloop :: sent length");
    //                //static_spi_handler::send_config(spi_peripheral::my_spi, config_info::config_to_send);
    ////                Serial.println("Superloop :: sent config");
    ////                Serial.print("message num : ");
    ////                Serial.print(i);
    ////                Serial.print(" = ");
    ////                Serial.println(spi_peripheral::controller_message[i]);
    //            }
    //            Serial.print("\n");
    
                
                simple_static_spi_handler::print_debug(spi_peripheral::debug_location);
                Serial.println("Superloop :: controller_message");
                print_message(spi_peripheral::controller_message, spi_peripheral::message_len);
                Serial.println("Superloop :: peripheral_message");
                print_message(spi_peripheral::peripheral_message, spi_peripheral::message_len);
                
                spi_peripheral::is_unread_message = false;
                time_of_last_message = millis();
    
    //            static_spi_handler::print_debug(spi_peripheral::debug_location);
    //            //print_config(config_info::config_to_send);
    //            static_spi_handler::print_message(spi_peripheral::controller_message, &exo_data, spi_peripheral::cmd);
    //            static_spi_handler::clear_message(spi_peripheral::controller_message);
          }
    
          if((millis() - time_of_last_message) > 1000)
          {
              Serial.println("+++++++++++++++++++++++++++++");
              time_of_last_message = millis();
          }
            
      }
    #endif  
#elif defined(ARDUINO_ARDUINO_NANO33BLE)
  #include <SPI.h>
  namespace config_info
  {
      uint8_t config_to_send[ini_config::number_of_keys] = {
            1,  // board name
            3,  // board version
            2,  // battery
            1,  // exo name
            1,  // exo side
            2,  // hip
            1,  // knee
            3,  // ankle
            1,  // hip gear
            1,  // knee gear
            1,  // ankle gear
            1,  // hip default controller
            1,  // knee default controller
            1,  // ankle default controller
            3,  // hip flip dir
            3,  // knee flip dir
            3,  // ankle flip dir
          };
      
  }

  #ifdef SIMPLE_EXAMPLE
      void setup()
    {
      Serial.begin(115200);
      while(!Serial);
      Serial.println("Setup : Serial Started");
      pinMode(coms_micro_pins::cs_pin, OUTPUT);
      digitalWrite(coms_micro_pins::cs_pin, HIGH);
      
      SPI.begin();
      SPI.transfer(0);//begin doesn't actually enable the system so send zero to nowhere to enable
      Serial.println("===========================================================");
      
    }
    
    void loop()
    {
        static uint8_t msg_num = 0;//  keep track of message num
        
        const uint8_t len = 4; // length of data to send
        uint8_t controller_message[len+1];  // add extra zero to be read at the start of the message by the other system
        uint8_t peripheral_message[len+1];
        uint8_t debug_location;// keep track of where we get without printing
  
        controller_message[0] = len;
        controller_message[1] = msg_num;//0x55;
        controller_message[2] = 0x0F;
        controller_message[3] = 0xF0;
        controller_message[4] = 0x00;//dummy value

        peripheral_message[0] = 0x00;
        peripheral_message[1] = 0x00;
        peripheral_message[2] = 0x00;
        peripheral_message[3] = 0x00;
        peripheral_message[4] = 0x00;

  
        Serial.println("Superloop :: Starting Transaction");
        Serial.print("Superloop :: msg_num : 0x");
        Serial.println(msg_num++,HEX);// print msg num
        SPI.beginTransaction(SPISettings(5000000, MSBFIRST, coms_micro_pins::spi_mode));  // teensy seems limited to 5 MHz
        
        digitalWrite(coms_micro_pins::cs_pin, LOW); // let the peripheral know you are talking to it
        
        for( int i = 0, j = 0; j<(len+1); i++)
        {
            peripheral_message[j] = SPI.transfer(utils::ff_to_fe(controller_message[i]));// send the data converting FF to FE so it isn't seen as error
            while(0xFF == peripheral_message[j])// if error recieved send it back so the other system knows
            {
              peripheral_message[j] = SPI.transfer(0xFF);
            }
            if((0 == j && 0 != peripheral_message[j])|| 0!=j)// ignore leading 0s
            {
              j++;
            }
        } 
        //uint8_t temp = SPI.transfer(0x00);
        digitalWrite(coms_micro_pins::cs_pin, HIGH);// tell the other system you are done with it
        SPI.endTransaction();
        
        Serial.println("Superloop :: controller_message");
        print_message(controller_message, len+1);
        Serial.println("Superloop :: peripheral_message");
        print_message(peripheral_message, len+1);
//        Serial.print("Superloop :: temp : 0x");
//        Serial.println(temp,HEX);
        delay(100);
    }
  #else
    void setup()
    {
      Serial.begin(115200);
      while(!Serial);
      Serial.println("Setup : Serial Started");
  //    simple_static_spi_handler::setup();
      pinMode(coms_micro_pins::cs_pin, OUTPUT);
      digitalWrite(coms_micro_pins::cs_pin, HIGH);
      
      SPI.begin();
      Serial.println("===========================================================");
      
    }
    
    void loop()
    {
        const uint8_t len = 4;
        uint8_t controller_message[len];
        uint8_t peripheral_message[len];
        uint8_t debug_location;
  
        static uint8_t start_val = 0xBB;
        
        controller_message[0] = len;
        controller_message[1] = 0xBB;
        controller_message[2] = 0x0B;
        controller_message[3] = 0xB0;
  
        Serial.println("Superloop :: Starting Transaction");
  
        SPI.beginTransaction(SPISettings(400000, MSBFIRST, coms_micro_pins::spi_mode));  // teensy seems limited to 26 MHz
        digitalWrite(coms_micro_pins::cs_pin, LOW);
        for( int i = 0; i<len; i++)
        {
            peripheral_message[i] = SPI.transfer(controller_message[i]);
        } 
  
        
        //debug_location = simple_static_spi_handler::controller_transaction(controller_message, peripheral_message, len);
  
        simple_static_spi_handler::print_debug(debug_location);
        Serial.println("Superloop :: controller_message");
        print_message(controller_message, len);
        Serial.println("Superloop :: peripheral_message");
        print_message(peripheral_message, len);
  
        delay(21000);
        
    }
    
  #endif
#endif
