#include "ExoBLE.h"
#include "Board.h"
#include "Utilities.h"

ExoBLE::ExoBLE(ExoData* data) : _data{data}
{
    pinMode(coms_micro_pins::ble_signal_pin, OUTPUT);
    digitalWrite(coms_micro_pins::ble_signal_pin, !coms_micro_pins::ble_signal_active);
    _old_buffer = new char[gatt_db.BUFFER_SIZE];
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
        BLE.setAdvertisedService(gatt_db.UARTService);
        gatt_db.UARTService.addCharacteristic(gatt_db.TXChar);
        gatt_db.UARTService.addCharacteristic(gatt_db.RXChar);
        BLE.addService(gatt_db.UARTService);

        gatt_db.RXChar.setEventHandler(BLEWritten, ble_rx::on_rx_recieved);
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

BleMessage ExoBLE::handle_updates() 
{
    BLE.poll();

    //check connection status
    if (_connected != BLE.connected()) 
    {
        _connected = BLE.connected();
        advertising_onoff(!_connected);
    }
    BleMessage empty_message = BleMessage();
    return empty_message;

    //check for update
    int buffer_length = gatt_db.RXChar.valueLength();
    char buffer[buffer_length];
    gatt_db.RXChar.readValue(buffer, buffer_length);
    if (!ble_rx::msg_queue.empty())
    {
        Serial.print(ble_rx::msg_queue.size());
        Serial.println(" Message need processing");
        BleMessage msg;
        BleMessage tmp = ble_rx::msg_queue.back();
        msg.copy(tmp);
        ble_rx::msg_queue.pop_back();
        return msg;
    }
    else
    {
        BleMessage empty_message = BleMessage();
        return empty_message;
    }
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
        for (int i=0; i<msg.expecting/8; i++)
        {
            Serial.print(msg.data[i]);
            Serial.print(", ");
        }
        Serial.println();
        //ble_rx::msg_queue.push_back(msg);
    }
}