#include "ComsMCU.h"

ComsMCU::ComsMCU(ExoData* data):_data{data}
{
    // TODO: Set battery type with config data
    _battery->init();
    _exo_ble = new ExoBLE(data);
    _exo_ble->setup();
}

void ComsMCU::handle_ble()
{
    bool got_new_message = _exo_ble->handle_updates();
    if (got_new_message)
    {
        BleMessage msg = pop_queue();
        if (msg.is_complete) 
        {
            _process_complete_gui_command(&msg);
        }
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
        BleMessage batt_msg = BleMessage();
        batt_msg.command = names::send_batt;
        batt_msg.expecting = 1;
        batt_msg.data[0] = batt_param;
        _exo_ble->send_message(batt_msg);
        then = now;
    }
}

void ComsMCU::_process_complete_gui_command(BleMessage* msg) 
{
    Serial.print("ComsMCU::_process_complete_gui_command->Got Command: ");
    BleMessage::print(*msg);

    switch (msg->command)
    {
    case names::start:
        handlers::start(_data);
        break;
    case names::stop:
        handlers::stop(_data);
        break;
    case names::cal_trq:
        handlers::cal_trq(_data);
        break;
    case names::cal_fsr:
        handlers::cal_fsr(_data);
        break;
    case names::assist:
        handlers::assist(_data);
        break;
    case names::resist:
        handlers::resist(_data);
        break;
    case names::motors_on:
        handlers::motors_on(_data);
        break;
    case names::motors_off:
        handlers::motors_off(_data);
        break;
    case names::mark:
        handlers::mark(_data);
        break;
    case names::new_fsr:
        handlers::new_fsr(_data);
        break;
    case names::new_trq:
        handlers::new_trq(_data);
        break;
    default:
        Serial.println("ComsMCU::_process_complete_gui_command->No handler for command!");
        break;
    }
}