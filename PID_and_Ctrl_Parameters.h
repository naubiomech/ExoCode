//All the needed parameters to set torque bias, PID ctrl, to enable the motors and to average the torque signals
int count = 0;

//Includes the PID library so we can utilize PID control
int PID_sample_time = 1;                                             //PID operates at 1000Hz, calling at a freq of 1 ms.
