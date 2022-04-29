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
        char* name_char;
        name.toCharArray(name_char, 6);
        BLE.setLocalName(name_char);
        BLE.setDeviceName(name_char);
        BLE.setAdvertisedService(_gatt_db.UARTService);
        _gatt_db.UARTService.addCharacteristic(_gatt_db.TXChar);
        _gatt_db.UARTService.addCharacteristic(_gatt_db.RXChar);
        BLE.addService(_gatt_db.UARTService);
        BLE.setEventHandler(BLEConnected, ExoBLE::onConnection);
        BLE.setEventHandler(BLEDisconnected, ExoBLE::onDisconnection);
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
        BLE.advertise();
    }
    else
    {
        BLE.stopAdvertise();
    }
}

BleMessage ExoBLE::handle_updates() 
{
    Serial.println("ExoBLE::handle_updates->Start");
    //check for update
    if (_gatt_db.RXChar.valueUpdated())
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
    Serial.println("ExoBLE::handle_updates->End");
}


/*
  Private Methods 
*/

void ExoBLE::onConnection(BLEDevice central)
{
    Serial.println("ExoBLE::onConnection->Connected");
    advertising_onoff(false);
}

void ExoBLE::onDisconnection(BLEDevice central)
{
    Serial.println("ExoBLE::onDisconnection->Disconnected");
    advertising_onoff(true);
}