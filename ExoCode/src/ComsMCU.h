/* XYZ

   Constructor:

   Chance Cuddeback 2022
*/
#ifndef COMSMCU_H
#define COMSMCU_H

#include "Arduino.h"
#include "ExoBLE.h"
#include "Battery.h"
#include "BleMessage.h"
#include "ParseIni.h"
#include "ExoData.h"
#include "BleMessageQueue.h"


class ComsMCU
{
    public:
        ComsMCU(ExoData* data, uint8_t* config_to_send);
        void handle_ble();
        void local_sample();
        void update_gui();
    private:
        void _process_complete_gui_command(BleMessage* msg);
        ExoBLE* _exo_ble;
        BleMessage _latest_gui_message = BleMessage();
        //Data
        ExoData* _data;
        //Battery
        _Battery* _battery;

        const float _battery_ewma_alpha = 0.1;
};

#endif