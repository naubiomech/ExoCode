#include "ComsMCU.h"
#include "StatusLed.h"
#include "StatusDefs.h"
#include "Time_Helper.h"

#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include <SPI.h>
#include <algorithm> // needed for std::find
#include "SPIMessageQueue.h"


ComsMCU::ComsMCU(ExoData* data, uint8_t* config_to_send)
:spi_handler(config_to_send, data)
,_data{data}
{
    switch (config_to_send[config_defs::battery_idx])
    {
    case (uint8_t)config_defs::battery::smart:
        _battery = new SmartBattery();
        break;
    case (uint8_t)config_defs::battery::dumb:
        _battery = new RCBattery();
        break;
    default:
        Serial.println("ERROR: ComsMCU::ComsMCU->Unrecognized battery type!");
        _battery = new RCBattery();
        break;
    }
    _battery->init();
    _exo_ble = new ExoBLE(data);
    _exo_ble->setup();
    
    bool have_valid_config = false; 

    // keep trying till we have a valid config, e.g. no zeros
    int spi_delay_ms = 100;
    while(!have_valid_config && (std::find(config_to_send, config_to_send + ini_config::number_of_keys, 0) != config_to_send + ini_config::number_of_keys))
    {
        Serial.println("ComsMCU::ComsMCU: waiting for config.");
        spi_handler.transaction(spi_cmd::send_config::id);
        delay(spi_delay_ms);
        have_valid_config = true;
    }
}

void ComsMCU::handle_ble()
{
    bool non_empty_ble_queue = _exo_ble->handle_updates();
    if (non_empty_ble_queue && !spi_queue::size())
    {
        BleMessage msg = ble_queue::pop();
        _process_complete_gui_command(&msg);
        spi_queue::push(&msg);
    }
}

void ComsMCU::local_sample()
{
    Time_Helper* t_helper = Time_Helper::get_instance();
    static const float context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(context);
    if (del_t > (BLE_times::_status_msg_delay/2)) 
    {
        static float filtered_value = _battery->get_parameter();
        float raw_battery_value = _battery->get_parameter();
        filtered_value = utils::ewma(raw_battery_value, filtered_value, _battery_ewma_alpha);
        _data->battery_value = filtered_value;
        del_t = 0;
    }   
}

void ComsMCU::update_spi()
{
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static const float spi_context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(spi_context);
    
    if (del_t > SPI_times::UPDATE_PERIOD)
    {
        spi_joint_pair_t spi_msg_pair = spi_queue::pop();
        if (0!=spi_msg_pair.spi_id)
        {
            spi_handler.transaction(spi_msg_pair.spi_id, spi_msg_pair.joint_id);
        }
        else
        {
            spi_handler.transaction(spi_cmd::send_data_exo::id);
        }
        del_t = 0;
    }
    
}

void ComsMCU::update_gui() 
{
    // Get real time data from ExoData and send to GUI
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static const float rt_context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(rt_context);

    if ((_data->status == status_defs::messages::trial_on) || 
    (_data->status == status_defs::messages::fsr_calibration) ||
    (_data->status == status_defs::messages::fsr_refinement) && 
    (del_t > BLE_times::_real_time_msg_delay))
    {
        //Serial.println("Sending rt data");
        static const float msg_context = t_helper->generate_new_context();
        static float time_since_last_message;
        time_since_last_message = t_helper->tick(msg_context);
        
        BleMessage rt_data_msg = BleMessage();
        rt_data_msg.command = ble_names::send_real_time_data;
        rt_data_msg.expecting = ble_command_helpers::get_length_for_command(rt_data_msg.command);
        // TODO: populate rt_data_msg and send
        rt_data_msg.data[0] = _data->right_leg.ankle.torque_reading;
        rt_data_msg.data[1] = 0.5;//_data->right_leg.ankle.controller.get_state(); TODO: Implement PJMC
        rt_data_msg.data[2] = _data->right_leg.ankle.controller.setpoint;
        rt_data_msg.data[3] = _data->left_leg.ankle.torque_reading;
        //TODO: Implement Mark Feature
        rt_data_msg.data[4] = 0.5;//_data->right_leg.ankle.controller.get_state(); TODO: Implement PJMC 
        rt_data_msg.data[5] = _data->left_leg.ankle.controller.setpoint;
        rt_data_msg.data[6] = 0;//_data->right_leg.toe_fsr;
        rt_data_msg.data[7] = 0;//_data->left_leg.toe_fsr;
        rt_data_msg.data[8] = time_since_last_message/1000.0;

        static const float send_context = t_helper->generate_new_context();
        static float send_del_t = 0;
        send_del_t = t_helper->tick(send_context);
        //_exo_ble->send_message(rt_data_msg);  
        send_del_t = t_helper->tick(send_context);
        //Serial.print("ComsMCU :: update_gui : Rt send time -> "); Serial.println(send_del_t);      

        del_t = 0;
    }

    // Periodically send status information
    static const float status_context = t_helper->generate_new_context();
    static float del_t_status = 0;
    del_t_status += t_helper->tick(status_context);
    if (del_t_status > BLE_times::_status_msg_delay)
    {
        // Send status data
        BleMessage batt_msg = BleMessage();
        batt_msg.command = ble_names::send_batt;
        batt_msg.expecting = ble_command_helpers::get_length_for_command(batt_msg.command);
        batt_msg.data[0] = _data->battery_value;
        _exo_ble->send_message(batt_msg);

        del_t_status = 0;
    }
}

void ComsMCU::_process_complete_gui_command(BleMessage* msg) 
{
    Serial.print("ComsMCU::_process_complete_gui_command->Got Command: ");
    BleMessage::print(*msg);

    switch (msg->command)
    {
    case ble_names::start:
        ble_handlers::start(_data, msg);
        break;
    case ble_names::stop:
        ble_handlers::stop(_data, msg);
        break;
    case ble_names::cal_trq:
        ble_handlers::cal_trq(_data, msg);
        break;
    case ble_names::cal_fsr:
        ble_handlers::cal_fsr(_data, msg);
        break;
    case ble_names::assist:
        ble_handlers::assist(_data, msg);
        break;
    case ble_names::resist:
        ble_handlers::resist(_data, msg);
        break;
    case ble_names::motors_on:
        ble_handlers::motors_on(_data, msg);
        break;
    case ble_names::motors_off:
        ble_handlers::motors_off(_data, msg);
        break;
    case ble_names::mark:
        ble_handlers::mark(_data, msg);
        break;
    case ble_names::new_fsr:
        ble_handlers::new_fsr(_data, msg);
        break;
    case ble_names::new_trq:
        ble_handlers::new_trq(_data, msg);
        break;
    default:
        Serial.println("ComsMCU::_process_complete_gui_command->No case for command!");
        break;
    }
}
#endif