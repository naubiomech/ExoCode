#include "ExoBLE.h"
#include "Utilities.h"
#include "Time_Helper.h"
#include "ComsLed.h"

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"
#define AT_COMMAND_TIMEOUT          2000   //milliseconds
#endif

ExoBLE::ExoBLE(ExoData* data) : _data{data}
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    , _ble{logic_micro_pins::cs_pin, logic_micro_pins::irq_pin, logic_micro_pins::rst_pin}
#endif
{
   ;
}

bool ExoBLE::setup() 
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        if (BLE.begin())
        {
            //Setup name
            String name = utils::remove_all_chars(BLE.address(), ':');
            name.remove(name.length() - MAC_ADDRESS_NAME_LENGTH);
            name = NAME_PREAMBLE + name;
            char name_char[name.length()];
            name.toCharArray(name_char, name.length()+1);
            const char* k_name_pointer = name_char;
            BLE.setLocalName(k_name_pointer);
            BLE.setDeviceName(k_name_pointer);
            //Configure service and start advertising
            BLE.setAdvertisedService(_gatt_db.UARTService);
            _gatt_db.UARTService.addCharacteristic(_gatt_db.TXChar);
            _gatt_db.UARTService.addCharacteristic(_gatt_db.RXChar);
            BLE.addService(_gatt_db.UARTService);

            _gatt_db.RXChar.setEventHandler(BLEWritten, ble_rx::on_rx_recieved);
            BLE.setConnectionInterval(6, 6);
            advertising_onoff(true);
            return true;
        }
        else
        {
            Serial.println("ExoBLE::setup->Failed to start BLE!");
            return false;
        }
    #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        if (!_ble.begin(0))
        {
            Serial.println(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
            return false;
        }
        Serial.println( F("OK!") );
        if (FACTORYRESET_ENABLE)
        {
            /* Perform a factory reset to make sure everything is in a known state */
            Serial.println(F("Performing a factory reset: "));
            if ( ! _ble.factoryReset() )
            {
                Serial.println(F("Couldn't factory reset"));
                return false;
            }
        }

        if ( !_ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
        {
            Serial.println( F("Callback requires at least 0.7.0") );
            while(true) {;}
        }
        
        /* Disable command echo from Bluefruit */
        _ble.echo(false);

        /* Disables the BLE libraries internal print statements */
        _ble.verbose(false);

        /* Get MAC Address from BLE Module, this address is static (by default) and is used in the name of the device */
        char mac_address[MAC_ADDRESS_TOTAL_LENGTH];
        _ble.atcommandStrReply(F("AT+BLEGETADDR"), mac_address, MAC_ADDRESS_TOTAL_LENGTH, AT_COMMAND_TIMEOUT);
        String name = utils::remove_all_chars(mac_address, ':');
        name.remove(name.length() - MAC_ADDRESS_NAME_LENGTH);
        // /* All Bluetooth names must include the preamble to connect to the GUI */
        name = NAME_PREAMBLE + name;
        // /* Package the name into the AT command */
        String change_name_command = "AT+GAPDEVNAME=" + name;
        char char_command[change_name_command.length()];
        change_name_command.toCharArray(char_command, change_name_command.length()+1);
        const __FlashStringHelper* name_command = reinterpret_cast<__FlashStringHelper*>(char_command);
        /* Set the name on the module using the packaged string */
        Serial.println(name_command);
        if (!_ble.sendCommandCheckOK(F(name_command))) 
        {
            Serial.println(F("Could not set device name"));
            return false;
        }

        /* Set event handler(s) */
        _ble.setConnectCallback(connection_callbacks::connected);
        _ble.setDisconnectCallback(connection_callbacks::disconnected);
        _ble.setBleUartRxCallback(ble_rx::on_rx_recieved);

        _ble.reset();

        _ble.setMode(BLUEFRUIT_MODE_DATA);
        
        /* Start Advertising */
        advertising_onoff(true);
        
        Serial.println("Finished Setup and Started Advertising!");
        return true;
    #endif
}

void ExoBLE::advertising_onoff(bool onoff) 
{
    if (onoff)
    {
        // Start Advertising
        Serial.println("Start Advertising");
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
            BLE.advertise();
            // turn the blue led off
            ComsLed* led = ComsLed::get_instance();
            uint8_t r, g, b;
            led->get_color(&r, &g, &b);
            led->set_color(r, g, 0);
        #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
            _ble.sendCommandCheckOK("AT+GAPSTARTADV");
        #endif
    }
    else
    {
        // Stop Advertising
        Serial.println("Stop Advertising");
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
            BLE.stopAdvertise();
            // turn the blue led on
            ComsLed* led = ComsLed::get_instance();
            uint8_t r, g, b;
            led->get_color(&r, &g, &b);
            led->set_color(r, g, 255);
        #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
            _ble.sendCommandCheckOK("AT+GAPSTOPADV");
        #endif
    }
}

bool ExoBLE::handle_updates() 
{
    static Time_Helper *t_helper = Time_Helper::get_instance();
    static float update_context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(update_context);

    if (del_t > BLE_times::_update_delay)
    {
        del_t = 0;
        #ifdef EXOBLE_DEBUG
        static float poll_context = t_helper->generate_new_context();
        static float poll_time = 0;
        static float connected_context = t_helper->generate_new_context();
        static float connected_time = 0;
        #endif

        // Poll for updates and check connection status
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
            BLE.poll(BLE_times::_poll_timeout);
            int32_t current_status = BLE.connected();
        #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
            #ifdef EXOBLE_DEBUG
            t_helper->tick(poll_context);
            //Serial.println("ExoBLE :: handle_updates : running update");
            #endif
            
            Serial.println("ExoBLE :: handle_updates : running update");
            _ble.update(0);

            #ifdef EXOBLE_DEBUG

            poll_time = t_helper->tick(poll_context);
            if (poll_time > 1000) {
                //Serial.print("ExoBLE :: handle_updates : poll time: "); Serial.println(poll_time);
                poll_time = 0;
            }
            t_helper->tick(connected_context);
            #endif

            int32_t current_status = connection_callbacks::is_connected;

            #ifdef EXOBLE_DEBUG
            connected_time = t_helper->tick(connected_context);
            if (connected_time > 1000) {
                //Serial.print("ExoBLE :: handle_updates : connected time: "); Serial.println(connected_time);
                connected_time = 0;
            }
            #endif
        #endif
        
        if (_connected == current_status) 
        {
            return ble_queue::size();
        }

        // The BLE connection status changed
        if (current_status < _connected)
        {
            // Disconnection
            #ifdef EXOBLE_DEBUG
            Serial.println("Disconnection");
            #endif
        }
        else if (current_status > _connected)
        {
            // Connection
            #ifdef EXOBLE_DEBUG
            Serial.println("Connection");
            #endif

        }
        advertising_onoff(current_status == 0);
        _connected = current_status;
            
    }
    return ble_queue::size();
}


void ExoBLE::send_message(BleMessage &msg)
{
    if (!this->_connected)
    {
        return; /* Don't bother sending anything if no one is listening */
    }
    #ifdef EXOBLE_DEBUG
    Serial.println("Exoble::send_message->Sending:");
    BleMessage::print(msg);
    #endif
    static const int k_preamble_length = 3;
    int max_payload_length = ((k_preamble_length + msg.expecting) * (MAX_PARSER_CHARACTERS + 1));
    byte buffer[max_payload_length];

    int bytes_to_send = _ble_parser.package_raw_data(buffer, msg);
    #if (ARDUINO_ARDUINO_NANO33BLE)
        _gatt_db.TXChar.writeValue(buffer, bytes_to_send);
    #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        _ble.writeBLEUart(buffer, bytes_to_send);
    #endif
}

#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
void ble_rx::on_rx_recieved(BLEDevice central, BLECharacteristic characteristic)
{
    static BleParser* parser = new BleParser();
    static BleMessage* msg = new BleMessage();

    char data[32] = {0};
    int len = characteristic.valueLength();
    characteristic.readValue(data, len);

        #ifdef EXOBLE_DEBUG
        Serial.print("On Rx Recieved: ");
        for (int i=0; i<len;i++)
        {
            Serial.print(data[i]);
            Serial.print(data[i], HEX);
            Serial.print(", ");
        }
        Serial.println();
        #endif

    msg = parser->handle_raw_data(data, len);
    if (msg->is_complete)
    {

        #ifdef EXOBLE_DEBUG
        Serial.print("on_rx_recieved->Command: ");
        Serial.println(msg->command);
        for (int i=0; i<msg->expecting; i++)
        {
            Serial.print(msg->data[i]);
            Serial.print(", ");
        }
        Serial.println();
        #endif

        ble_queue::push(msg);
    }
}

#elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
void ble_rx::on_rx_recieved(char data[], uint16_t len)
{
    static BleParser* parser = new BleParser();
    static BleMessage* msg = new BleMessage();

        #ifdef EXOBLE_DEBUG
        Serial.print("On Rx Recieved: ");
        for (int i=0; i<len;i++)
        {
            Serial.print(data[i]);
            Serial.print(data[i], HEX);
            Serial.print(", ");
        }
        Serial.println();
        #endif

    for (int i=0; i<len; i++)
    {
        char working_char = data[i];
        msg = parser->handle_raw_data(&working_char, 1);
        if (msg->is_complete)
        {
            ble_queue::push(msg);
            msg->clear();
        }
    }
}

void connection_callbacks::connected(void)
{
    connection_callbacks::is_connected = true;
}
void connection_callbacks::disconnected(void)
{
    connection_callbacks::is_connected = false;
}

#endif