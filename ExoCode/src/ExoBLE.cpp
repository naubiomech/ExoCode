#include "ExoBLE.h"
#include "Board.h"
#include "Utilities.h"

ExoBLE::ExoBLE(ExoData* data) : _data{data}
{
    pinMode(coms_micro_pins::ble_signal_pin, OUTPUT);
    digitalWrite(coms_micro_pins::ble_signal_pin, !coms_micro_pins::ble_signal_active);
    _old_buffer = new char[_gatt_db.BUFFER_SIZE];
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
        advertising_onoff(true);
    }
    else
    {
        Serial.println("ExoBLE::setup->Failed to start BLE!");
        while(true){;}
    }
}

void ExoBLE::advertising_onoff(bool onoff) 
{
    if (onoff)
    {
        Serial.println("Advertising");
        BLE.advertise();
        digitalWrite(coms_micro_pins::ble_signal_pin, !coms_micro_pins::ble_signal_active);
    }
    else
    {
        Serial.println("Stopped Advertising");
        BLE.stopAdvertise();
        digitalWrite(coms_micro_pins::ble_signal_pin, coms_micro_pins::ble_signal_active);
    }
}

BleMessage ExoBLE::handle_updates() 
{
    BLE.poll();

    //check connection status
    if (_connected != BLE.connected()) 
    {
        _connected = BLE.connected();
        advertising_onoff(!_connected);
    }

    //check for update
    int buffer_length = _gatt_db.RXChar.valueLength();
    char buffer[buffer_length];
    _gatt_db.RXChar.readValue(buffer, buffer_length);
    if (_connected && !utils::elements_are_equal(_old_buffer, buffer, buffer_length))
    {
        utils::set_elements_equal(_old_buffer, buffer, buffer_length);
        Serial.print("ExoBLE::handle_updates->Got Data: ");
        Serial.println(buffer_length);
        for (int i=0; i<buffer_length; i++)
        {
                Serial.print(buffer[i]);
                Serial.print("\t");
                int int_cast = (int) buffer[i];
                Serial.print(int_cast);
                Serial.print("\t");
        }
        Serial.println();
        return _ble_parser.handle_raw_data(buffer, buffer_length);
    }
    BleMessage empty_message = BleMessage();
    return empty_message;
}