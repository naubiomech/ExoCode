#include "ExoBLE.h"
#include "Utilities.h"

ExoBLE::ExoBLE(ExoData* data) : _data{data}
{

}

bool ExoBLE::setup() 
{
    if (BLE.begin())
    {
        String name = utils::remove_all_chars(BLE.address(), ':');
        name.remove(6);
        name = "EXOBLE_" + name;
        Serial.print("ExoBLE::setup()"); 
        Serial.print("Name: ");
        Serial.println(name); 
        char name_char[13];
        name.toCharArray(name_char, 13+1);
        for (int i=0; i<13; i++)
            Serial.print(name_char[i]);
        Serial.println();
        const char* n = name_char;
        BLE.setLocalName(n);
        BLE.setDeviceName(n);
        BLE.setAdvertisedService(_gatt_db.UARTService);
        _gatt_db.UARTService.addCharacteristic(_gatt_db.TXChar);
        _gatt_db.UARTService.addCharacteristic(_gatt_db.RXChar);
        BLE.addService(_gatt_db.UARTService);
        advertising_onoff(true);
    }
    else
    {
        Serial.println("ExoBLE::setup->Failed to start BLE!");
    }
}

void ExoBLE::advertising_onoff(bool onoff) 
{
    if (onoff)
    {
        Serial.println("Advertising");
        BLE.advertise();
    }
    else
    {
        Serial.println("Stopped Advertising");
        BLE.stopAdvertise();
    }
}

BleMessage ExoBLE::handle_updates() 
{
    //check connection status
    if (_connected != BLE.connected()) 
    {
        _connected = BLE.connected();
        advertising_onoff(!_connected);
    }

    //check for update
    if (_connected && _gatt_db.RXChar.valueUpdated())
    {
        Serial.println("ExoBLE::handle_updates->Updated");
        //parse new data
        int buffer_length = _gatt_db.RXChar.valueLength();
        char buffer[buffer_length];
        _gatt_db.RXChar.readValue(buffer, buffer_length);
        //return properly formatted BleMessage
        Serial.println("ExoBLE::handle_updates->Got Data: ");
        for (int i=0; i<buffer_length; i++)
        {
                Serial.print(buffer[i]);
                Serial.print("\t");
        }
        Serial.println();
        return _ble_parser.handle_raw_data(buffer, buffer_length);
    }
    BleMessage empty_message = BleMessage();
    return empty_message;
}