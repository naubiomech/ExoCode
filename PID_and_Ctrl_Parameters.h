//All the needed parameters to set torque bias, PID ctrl, to enable the motors and to average the torque signals
double Tcal_LL = 0;
double Tcal_RL = 0;
int count = 0;

double T_act_LL, T_act_RL;
int Vol_LL, Vol_RL;
const unsigned int motor_LL_pin = A21;                                     //analogwrite pin
const unsigned int motor_RL_pin = A22;                                     //analogwrite pin

double kp_LL = 800;
double ki_LL = 0;
double kd_LL = 3;
double KF_LL = 1;
double kp_RL = 800;
double ki_RL = 0;
double kd_RL = 3;
double KF_RL = 1;

//Includes the PID library so we can utilize PID control
int PID_sample_time = 1;                                             //PID operates at 1000Hz, calling at a freq of 1 ms.
double PID_Setpoint_LL, Input_LL, Output_LL;                             //Initializes the parameters for the PID controll
double PID_Setpoint_RL, Input_RL, Output_RL;                             //Initializes the parameters for the PID controll
PID PID_LL(&Input_LL, &Output_LL, &PID_Setpoint_LL, kp_LL, ki_LL, kd_LL, DIRECT); //Sets up PID for the right leg
PID PID_RL(&Input_RL, &Output_RL, &PID_Setpoint_RL, kp_RL, ki_RL, kd_RL, DIRECT); //Sets up PID for the right leg

// The setpoint (torque reference) for the left and right ankle
double Setpoint_Ankle_RL, Setpoint_Ankle_RL_Pctrl;
double Setpoint_Ankle_LL, Setpoint_Ankle_LL_Pctrl;

double Previous_Setpoint_Ankle_RL = 0;
double Previous_Setpoint_Ankle_LL = 0;

// Pointers to easily work on the setpoint
double*p_Setpoint_Ankle_LL = &Setpoint_Ankle_LL;
double*p_Setpoint_Ankle_RL = &Setpoint_Ankle_RL;

double*p_Setpoint_Ankle_LL_Pctrl = &Setpoint_Ankle_LL_Pctrl;
double*p_Setpoint_Ankle_RL_Pctrl = &Setpoint_Ankle_RL_Pctrl;

// Setpoint early stance
double Setpoint_earlyStance_LL = .25 * Setpoint_Ankle_LL;
double Setpoint_earlyStance_RL = .25 * Setpoint_Ankle_RL;
