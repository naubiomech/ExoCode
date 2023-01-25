#ifndef GATT_DB_H
#define GATT_DB_H


#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
#include "Arduino.h"
#include "ArduinoBLE.h"


class GattDb
{
    private:
        const uint8_t BUFFERS_FIXED_LENGTH = false;
    public:
        const uint8_t BUFFER_SIZE = 128;
        //https://stackoverflow.com/questions/10052135/expected-identifier-before-string-constant
        /* Weirdness with initializer lists, had to use curly braces. */
        BLEService UARTService{"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"};
        BLECharacteristic TXChar{"6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLERead | BLENotify | BLEBroadcast,             BUFFER_SIZE, BUFFERS_FIXED_LENGTH};
        BLECharacteristic RXChar{"6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse | BLEWrite | BLENotify, BUFFER_SIZE, BUFFERS_FIXED_LENGTH};
        /*
            Payton 1/4/23
            defining new characteristics for the board info 
        */
        // Service and Characteristics for exo info
        BLEService UARTServiceDeviceInfo{"e0271458-8c6a-11ed-a1eb-0242ac120002"}; // Serivce UUID for sending exo data
        BLECharacteristic PCBChar{"e0271459-8c6a-11ed-a1eb-0242ac120002", BLERead, BUFFER_SIZE, BUFFERS_FIXED_LENGTH}; // Characteristic for pcb
        BLECharacteristic FirmwareChar{"e0271460-8c6a-11ed-a1eb-0242ac120002", BLERead, BUFFER_SIZE, BUFFERS_FIXED_LENGTH}; // Characteristic for firmware
        BLECharacteristic DeviceChar{"e0271461-8c6a-11ed-a1eb-0242ac120002", BLERead, BUFFER_SIZE, BUFFERS_FIXED_LENGTH}; // Characteristic for device
};

#endif
#endif
