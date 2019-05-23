#ifndef PARAMETERS_HEADER
#define PARAMETERS_HEADER
// ===== FSR Parameters =====
// To set FSR bias and to identify the states
double fsrLongCurrent = 0;
double fsrShortCurrent = 0;

// Single board small

double doubleFSR = 0;
int intFSR = 0;

int FSR_FIRST_Cycle = 1;
int FSR_CAL_FLAG = 0;

double base_1, base_2;

double FSR_Sensors_type = 40;

// ===== Filter Parameters =====
//To filter the reference (not used right now but still available)
double a_vect[3] = {1.0, -1.864734299516908, 0.869308501948704};
double *p_a_vect = a_vect;
double b_vect[3] = {0.001143550607949, 0.002287101215898, 0.001143550607949};
double *p_b_vect = b_vect;
int flag = 0;
float filterVal = .8;
double smoothedVal = 0;
int clean_flag = 1;

// ===== Shaping Parameters =====
//Paramenters used to shape the reference
double Ts = 0.001;
// the smoothing value, i.e. the sigmoind number of steps as a function of the EXO state
double N3 = 200;
double N2 = 4;
double N1 = 4;

int change = 1;
int just_first_time = 1;

// ===== State Machine Parameters =====
double state_counter_th = 3;
double step_time_length = 150;

// ===== PID and CTRL Parameters =====
//All the needed parameters to set torque bias, PID ctrl, to enable the motors and to average the torque signals
int count = 0;

//Includes the PID library so we can utilize PID control
int PID_sample_time = 1;                                             //PID operates at 1000Hz, calling at a freq of 1 ms.

// ===== Memory Addrss Parameters =====
//To store in memory. These are the address of the EEPROM of the Teensy
int address_params = 54;
int flag_save_EEPROM = 0;

// ===== Calibrate and read Parameters =====
double p[4] = {0.0787, -0.8471, 20.599, -22.670};

double p_prop[3] = {128.1, -50.82, 22.06};

double FSR_Ratio;

// ===== Proportional Control Parameters =====
double Max_Prop = 25;
double Min_Prop = 0;

// ===== Auto KF Parameters =====
double max_ERR = 0.20;
double min_ERR = -0.20;

// ===== IMU Parameters =====
const int BNO055_SAMPLERATE_DELAY_MS = 100;
const double stability_trq_gain = 0.01;

// ===== Torque Speed Adjust Parameters =====
const int n_step_baseline = 6;


#endif
