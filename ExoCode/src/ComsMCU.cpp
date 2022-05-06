#include "ComsMCU.h"

ComsMCU::ComsMCU(ExoData* data):_data{data}
{
     //TODO: Use config data to set battery class
    _exo_ble = new ExoBLE(data);
    _exo_ble->setup();
}

void ComsMCU::handle_ble()
{
    BleMessage* msg = _exo_ble->handle_updates();
    if(msg->is_complete)
    {
        Serial.println("ComsMCU::handle_ble->Got complete message");
        _process_complete_gui_command(msg);
    }
}

void ComsMCU::local_sample()
{
    // Get the data that the ComsMCU is responsible for collecting

}

void ComsMCU::update_gui() 
{
    // Get real time data from ExoData

    // Send over bluetooth


    // Incrementally send status information
    static float then = millis();
    float now = millis();
    if ((now-then) > _status_millis)
    {
        // Send status data
        float batt_param = _battery->get_parameter();
        Serial.print("ComsMCU::update_gui->Sending Battery Data: ");
        Serial.println(batt_param);
        BleMessage batt_msg = BleMessage();
        Serial.print("ComsMCU::update_gui->BleMessage Addr: ");
        Serial.println(reinterpret_cast<long unsigned int>(&batt_msg));
        batt_msg.command = names::send_batt;
        batt_msg.expecting = 1;
        batt_msg.data[0] = batt_param;
        Serial.print("ComsMCU::update_gui->Put in: ");
        Serial.println(batt_msg.data[0]);
        _exo_ble->send_message(batt_msg);
        then = now;
    }
}

void ComsMCU::_process_complete_gui_command(BleMessage* msg) 
{
    //Update Data

    //Process Command

    Serial.print("ComsMCU::_process_complete_gui_command->Got Command: ");
    Serial.print(msg->command);
    Serial.print("\t");
    Serial.println(msg->expecting);
    for (int i = 0; i < (msg->expecting); i++)
    {
        Serial.print(msg->data[i]);
        Serial.print("\t");
    }
    Serial.println();
}
