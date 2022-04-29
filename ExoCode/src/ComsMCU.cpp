#include "ComsMCU.h"

ComsMCU::ComsMCU(ExoData* data):_data{data}
{
     //TODO: Use config data to set battery class
    _exo_ble = new ExoBLE(data);
    _exo_ble->setup();
}

void ComsMCU::handle_ble()
{
    Serial.println("ComsMCU::handle_ble->Start");
    BleMessage msg = _exo_ble->handle_updates();
    Serial.println("ComsMCU::handle_ble->Post handle");
    Serial.print(msg.command); Serial.print("\t");
    Serial.print(msg.expecting);
    for (auto i = msg.data.begin(); i != msg.data.end(); i++)
        Serial.print((float)i);
    Serial.println();
    if(msg.is_complete)
    {
        Serial.println("ComsMCU::handle_ble->Pre process");
        _process_complete_gui_command(msg);
        Serial.println("ComsMCU::handle_ble->Post process");
    }
    Serial.println("ComsMCU::handle_ble->End");
}

void ComsMCU::local_sample()
{
    
}

void ComsMCU::update_gui() 
{
    // Get real time data from ExoData

    // Send over bluetooth

}

void ComsMCU::_process_complete_gui_command(BleMessage msg) 
{
    //Update Data

    //Process Command

    Serial.print("ComsMCU::_process_complete_gui_command->Got Command: ");
    Serial.print(msg.command);
    Serial.print("\t");
    Serial.print(msg.expecting);
    for (int i = 0; i < msg.expecting; i++)
    {
        Serial.print(msg.data[i]);
        Serial.print("\t");
    }
    Serial.println();
}
