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

#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include "Arduino.h"
#include "ArduinoBLE.h"
#include "BleParser.h"
#include "GattDb.h"
#include "BleMessage.h"
#include "ExoData.h"
#include "BleMessageQueue.h"


class ExoBLE 
{
    public:
        
        ExoBLE(ExoData* data);
        bool setup();
        static void advertising_onoff(bool onoff);
        bool handle_updates();
        void send_message(BleMessage &msg);

    private:
        bool _connected = false;
        ExoData* _data;
        
        GattDb _gatt_db = GattDb();
        BleParser _ble_parser = BleParser();
};

namespace ble_rx
{
    void on_rx_recieved(BLEDevice central, BLECharacteristic characteristic);
}

#endif
#endif