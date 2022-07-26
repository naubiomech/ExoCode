/* Class to handle all bluetooth work. This include initialization,
   advertising, connection, and data transfer. The class passes data to a seperate class to
   communicate that information over SPI to the controls microcontroller.

   Constructor: Default

   setup(char* name) initialized the GATT database and attributes callbacks
   advertise() change the status of advertising
   send_command_message(char command_char, double* data_to_send, int number_to_send)

   Chance Cuddeback 2022
*/


#ifndef EXOBLE_H
#define EXOBLE_H

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    #include <SPI.h>
    #include "Adafruit_BLE.h"
    #include "Adafruit_BluefruitLE_SPI.h"

    #define NAME_PREAMBLE               "EXOBLE_"
    #define MAC_ADDRESS_TOTAL_LENGTH    17
    #define MAC_ADDRESS_NAME_LENGTH     6
    #define MAX_PARSER_CHARACTERS       8
#elif defined(ARDUINO_ARDUINO_NANO33BLE)
    #include "ArduinoBLE.h"
#endif

#include <Arduino.h>
#include "BleParser.h"
#include "GattDb.h"
#include "BleMessage.h"
#include "ExoData.h"
#include "BleMessageQueue.h"
#include "Board.h"


class ExoBLE 
{
    public:
        ExoBLE(ExoData* data);
        bool setup();
        void advertising_onoff(bool onoff);
        bool handle_updates();
        void send_message(BleMessage &msg);

    private:
        int _connected = 0;
        ExoData* _data;
        
        #if defined(ARDUINO_ARDUINO_NANO33BLE)
            GattDb _gatt_db = GattDb();
        
        #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
            Adafruit_BluefruitLE_SPI _ble;//(logic_micro_pins::cs_pin, logic_micro_pins::irq_pin, -1);
        #endif
        BleParser _ble_parser = BleParser();
};

namespace ble_rx
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE)
        void on_rx_recieved(BLEDevice central, BLECharacteristic characteristic);
    #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        void on_rx_recieved(char data[], uint16_t len);
    #endif
}

#endif