#ifndef PILOGGER_H
#define PILOGGER_H

#include "ExoData.h"

class PiLogger 
{
    public: 
    PiLogger(ExoData* data) : _data(data) {}

    void sendUpdate()
    {
        _start();

        _print_tab("LTFsr", _data->left_leg.toe_fsr);
        _print_tab("LTSta", _data->left_leg.toe_stance);
        _print_tab("LJTor", _data->left_leg.ankle.torque_reading);
        _print_tab("LJPos", _data->left_leg.ankle.joint_position);
        _print_tab("LJVel", _data->left_leg.ankle.joint_velocity);
        _print_tab("LMPos", _data->left_leg.ankle.motor.p);
        _print_tab("LMVel", _data->left_leg.ankle.motor.v);
        _print_tab("LMCur", _data->left_leg.ankle.motor.i);
        _print_tab("LMCom", _data->left_leg.ankle.motor.last_command);

        _print_tab("LTFsr", _data->right_leg.toe_fsr);
        _print_tab("LTSta", _data->right_leg.toe_stance);
        _print_tab("LJTor", _data->right_leg.ankle.torque_reading);
        _print_tab("LJPos", _data->right_leg.ankle.joint_position);
        _print_tab("LJVel", _data->right_leg.ankle.joint_velocity);
        _print_tab("LMPos", _data->right_leg.ankle.motor.p);
        _print_tab("LMVel", _data->right_leg.ankle.motor.v);
        _print_tab("LMCur", _data->right_leg.ankle.motor.i);
        _print_tab("LMCom", _data->right_leg.ankle.motor.last_command);

        _print_tab("Error", _data->error_code);
        _print_tab("ErJID", _data->error_joint_id);
        _print_tab("Status", _data->get_status());

        _end();
    }

    private:
    ExoData* _data;

    String _format(String name, float value)
    {
        return name+":"+String(value);
    }

    void _print_tab(String name, float value)
    {
        logger::print(_format(name, value)+"\t");
    }

    void _start()
    {
        logger::println("piLoggerStart");
    }
    
    void _end()
    {
        logger::println("piLoggerEnd");
    }
    
};

#endif