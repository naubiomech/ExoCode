/**
 * @file ExoBLE.h
 * @author Chance Cuddeback
 * @brief Class to handle all bluetooth work. This include initialization,
 * advertising, connection, and data transfer. 
 * @date 2022-08-22
 * 
 */


#ifndef EXOBLE_H
#define EXOBLE_H

//#define EXOBLE_DEBUG
#define MAX_PARSER_CHARACTERS       8
#define NAME_PREAMBLE               "EXOBLE_"
#define MAC_ADDRESS_TOTAL_LENGTH    17
#define MAC_ADDRESS_NAME_LENGTH     6

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    #include <SPI.h>
    #include "Adafruit_BLE.h"
    #include "Adafruit_BluefruitLE_SPI.h"
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

// #if defined(ARDUINO_ARDUINO_NANO33BLE)

class ExoBLE 
{
    public:
        /**
         * @brief Construct a new Exo B L E object
         * 
         * @param data A reference to the ExoData object
         */
        ExoBLE(ExoData* data);
        /**
         * @brief Sets GATT DB, device name, and begins advertising. 
         * 
         * @return true If the initialization succeeded
         * @return false If the initialization failed
         */
        bool setup();

        /**
         * @brief Starts and stops advertising.
         * 
         * @param onoff True to begin advertising, false to stop. 
         */
        void advertising_onoff(bool onoff);

        /**
         * @brief Checks for changes in the connection status and polls for BLE events
         * 
         * @return true If there is data waiting in the message queue
         * @return false False if there is no data in the message queue
         */
        bool handle_updates();

        /**
         * @brief Send a BLE message using the Nordic UART Service. The data is serialized with the parser object. 
         * 
         * @param msg The message that you would like to send.
         */
        void send_message(BleMessage &msg);

    private:

        // BLE connection state
        int _connected = 0;

        // A reference to ExoData
        ExoData* _data;
        

        #if defined(ARDUINO_ARDUINO_NANO33BLE)
            // The Gatt database which defines the services and characteristics
            GattDb _gatt_db = GattDb();
        
        #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
            // The class to interact with the Adafruit_Bluefruit_SPI_Friend
            Adafruit_BluefruitLE_SPI _ble;
        #endif

        // The parser used to serialize and deserialize the BLE data
        BleParser _ble_parser = BleParser();
};

/**
 * @brief Holds the callbacks for the data reception
 * 
 */
namespace ble_rx
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE)
        void on_rx_recieved(BLEDevice central, BLECharacteristic characteristic);
    #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        void on_rx_recieved(char data[], uint16_t len);
    #endif
}

/**
 * @brief Holds the callbacks for the connection events, only used if using the Adafruit_Bluefruit_SPI_Friend
 * 
 */
namespace connection_callbacks
{
    static bool is_connected = false;
    #if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        void connected(void);
        void disconnected(void);
    #endif
}
// #endif
#endif