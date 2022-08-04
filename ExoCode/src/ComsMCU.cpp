#include "ComsMCU.h"
#include "StatusLed.h"
#include "Time_Helper.h"

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
    // Get real time data from ExoData and send to GUI
    if (_data->status == status_led_defs::messages::trial_on)
    {
        Time_Helper* t_helper = Time_Helper::get_instance();
        static const float timer_context = t_helper->generate_new_context();
        static float del_t;
        del_t = t_helper->tick(timer_context);
        
        BleMessage rt_data_msg = BleMessage();
        rt_data_msg.command = names::send_real_time_data;
        rt_data_msg.expecting = ble_command_helpers::get_length_for_command(rt_data_msg.command);
        // TODO: populate rt_data_msg and send
        rt_data_msg.data[0] = _data->right_leg.ankle.torque_reading;
        rt_data_msg.data[1] = 0.5;//_data->right_leg.ankle.controller.get_state(); TODO: Implement PJMC
        rt_data_msg.data[2] = _data->right_leg.ankle.controller.setpoint;
        rt_data_msg.data[3] = _data->left_leg.ankle.torque_reading;
        //TODO: Implement Mark Feature
        rt_data_msg.data[4] = 0.5;//_data->right_leg.ankle.controller.get_state(); TODO: Implement PJMC 
        rt_data_msg.data[5] = _data->left_leg.ankle.controller.setpoint;
        rt_data_msg.data[6] = _data->right_leg.toe_fsr;
        rt_data_msg.data[7] = _data->left_leg.toe_fsr;
        rt_data_msg.data[8] = del_t;

        _exo_ble->send_message(rt_data_msg);        
    }

    // Periodically send status information
    static float then = millis();
    float now = millis();
    if ((now-then) > _status_millis)
    {
        // Send status data
        float batt_param = _battery->get_parameter();
        BleMessage batt_msg = BleMessage();
        batt_msg.command = names::send_batt;
        batt_msg.expecting = ble_command_helpers::get_length_for_command(batt_msg.command);
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