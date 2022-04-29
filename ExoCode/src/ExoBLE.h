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

#include "Arduino.h"
#include "ArduinoBLE.h"
#include "BleParser.h"
#include "GattDb.h"
#include "BleMessage.h"
#include "ExoData.h"


class ExoBLE 
{
    public:
        ExoBLE(ExoData* data);
        bool setup();
        static void advertising_onoff(bool onoff);
        BleMessage handle_updates();

    private:
        ExoData* _data;
        GattDb _gatt_db;
        BleParser _ble_parser = BleParser();
        static void onConnection(BLEDevice central);
        static void onDisconnection(BLEDevice central);
};

#endif
