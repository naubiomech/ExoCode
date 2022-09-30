#include "ComsMCU.h"
#include "StatusLed.h"
#include "StatusDefs.h"
#include "Time_Helper.h"
#include "UARTHandler.h"
#include "UART_commands.h"
#include "UART_msg_t.h"
#include "Config.h"

#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

ComsMCU::ComsMCU(ExoData* data, uint8_t* config_to_send):_data{data}
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


    uint8_t rt_data_len = 0;
    switch (config_to_send[config_defs::exo_name_idx])
    {
        case (uint8_t)config_defs::exo_name::bilateral_ankle:
            rt_data_len = UART_rt_data::BILATERAL_ANKLE_RT_LEN;
            break;
        case (uint8_t)config_defs::exo_name::bilateral_hip:
            rt_data_len = UART_rt_data::BILATERAL_HIP_RT_LEN;
            break;
        case (uint8_t)config_defs::exo_name::bilateral_hip_ankle:
            rt_data_len = UART_rt_data::BILATERAL_HIP_ANKLE_RT_LEN;
            break;
        default:
            rt_data_len = 8;
            break;
    }
    UART_rt_data::msg_len = rt_data_len;
    // Serial.print("ComsMCU::ComsMCU->rt_data_len: "); Serial.println(rt_data_len);
    
}

void ComsMCU::handle_ble()
{
    bool non_empty_ble_queue = _exo_ble->handle_updates();
    if (non_empty_ble_queue)
    {
        BleMessage msg = ble_queue::pop();
        _process_complete_gui_command(&msg);
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
        filtered_value = utils::ewma(raw_battery_value, filtered_value, k_battery_ewma_alpha);
        _data->battery_value = filtered_value;
        del_t = 0;
    }
}

void ComsMCU::update_UART()
{
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static const float _context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(_context);
    
    if (true)//(del_t > UART_times::UPDATE_PERIOD)
    {
        UARTHandler* handler = UARTHandler::get_instance();
        UART_msg_t msg = handler->poll(UART_times::COMS_MCU_TIMEOUT);
        if (msg.command) 
        {
            // UART_msg_t_utils::print_msg(msg);
            UART_command_utils::handle_msg(handler, _data, msg);
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
        // static const float msg_context = t_helper->generate_new_context();
        // static float time_since_last_message;
        // time_since_last_message = t_helper->tick(msg_context);
        // if (time_since_last_message > k_time_threshold)
        // {
        //     time_since_last_message = 0;
        // }
        
        BleMessage rt_data_msg = BleMessage();
        rt_data_msg.command = ble_names::send_real_time_data;
        if (UART_rt_data::msg.len == UART_rt_data::msg_len) 
        {
            rt_data_msg.expecting = UART_rt_data::msg.len;
            for (int i = 0; i < UART_rt_data::msg.len; i++)
            {
                rt_data_msg.data[i] = UART_rt_data::msg.data[i];
            }
            rt_data_msg.data[rt_data_msg.expecting++] = 0;//time_since_last_message/1000.0;
            BleMessage::print(rt_data_msg);

            _exo_ble->send_message(rt_data_msg);
        }
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