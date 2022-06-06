#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include "ExoBLE.h"
#include "Board.h"
#include "Utilities.h"

ExoBLE::ExoBLE(ExoData* data) : _data{data}
{
    pinMode(coms_micro_pins::ble_signal_pin, OUTPUT);
    digitalWrite(coms_micro_pins::ble_signal_pin, !coms_micro_pins::ble_signal_active);
}

bool ExoBLE::setup() 
{
    if (BLE.begin())
    {
        //Setup name
        String name = utils::remove_all_chars(BLE.address(), ':');
        name.remove(6);
        name = "EXOBLE_" + name;
        char name_char[13];
        name.toCharArray(name_char, 13+1);
        const char* n = name_char;
        BLE.setLocalName(n);
        BLE.setDeviceName(n);
        //Configure service and start advertising
        BLE.setAdvertisedService(_gatt_db.UARTService);
        _gatt_db.UARTService.addCharacteristic(_gatt_db.TXChar);
        _gatt_db.UARTService.addCharacteristic(_gatt_db.RXChar);
        BLE.addService(_gatt_db.UARTService);

        _gatt_db.RXChar.setEventHandler(BLEWritten, ble_rx::on_rx_recieved);
        BLE.setConnectionInterval(6, 6);
        advertising_onoff(true);
    }
    else
    {
        Serial.println("ExoBLE::setup->Failed to start BLE!");
        while(true) {;}
    }
}

void ExoBLE::advertising_onoff(bool onoff) 
{
    if (onoff)
    {
        BLE.advertise();
        digitalWrite(coms_micro_pins::ble_signal_pin, !coms_micro_pins::ble_signal_active);
    }
    else
    {
        BLE.stopAdvertise();
        digitalWrite(coms_micro_pins::ble_signal_pin, coms_micro_pins::ble_signal_active);
    }
}

bool ExoBLE::handle_updates() 
{
    BLE.poll();

    //check connection status
    if (_connected != BLE.connected()) 
    {
        _connected = BLE.connected();
        advertising_onoff(!_connected);
    }

    return queue_size();
}


void ExoBLE::send_message(BleMessage &msg)
{
    // Serial.print("Address: ");
    // Serial.println(reinterpret_cast<long unsigned int>(&msg));
    const int max_chars = 8;
    int max_payload_length = ((3 + msg.expecting) * (max_chars + 1));
    byte buffer[max_payload_length];
    // Serial.print("ExoBLE::send_message->Data: ");
    // Serial.println(msg.data[0]);
    int bytes_to_send =  _ble_parser.package_raw_data(buffer, msg);
    _gatt_db.TXChar.writeValue(buffer, bytes_to_send);
}

void ble_rx::on_rx_recieved(BLEDevice central, BLECharacteristic characteristic)
{
    static BleParser* parser = new BleParser();

    char data[32] = {0};
    int len = characteristic.valueLength();
    characteristic.readValue(data, len);
    Serial.print("On Rx Recieved: ");
    for (int i=0; i<len;i++)
    {
        Serial.print(data[i]);
        Serial.print(data[i], HEX);
        Serial.print(", ");
    }
    Serial.println();
    BleMessage msg = parser->handle_raw_data(data, len);
    if (msg.is_complete)
    {
        Serial.print("on_rx_recieved->Command: ");
        Serial.println(msg.command);
        for (int i=0; i<msg.expecting; i++)
        {
            Serial.print(msg.data[i]);
            Serial.print(", ");
        }
        Serial.println();
        push_queue(&msg);
    }
}

#endif