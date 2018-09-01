#ifndef PARAMETERS_HEADER
#define PARAMETERS_HEADER

// Selects the board to compile for
#define IMU_BOARD

bool FLAG_PRINT_TORQUES = false;
bool FLAG_PID_VALS = false;
bool FLAG_TWO_TOE_SENSORS = false;

const unsigned int zero = 2048;//1540;

// ===== FSR Parameters =====
// To set FSR bias and to identify the states
const double FSR_Sensors_type = 40;

// ===== Shaping Parameters =====
//Paramenters used to shape the reference
const double Ts = 0.001;

// ===== State Machine Parameters =====
const double state_counter_th = 3;
const double step_time_length = 150;

// ===== PID and CTRL Parameters =====
//Includes the PID library so we can utilize PID control
const int PID_sample_time = 1;                                             //PID operates at 1000Hz, calling at a freq of 1 ms.

// ===== Memory Addrss Parameters =====
//To store in memory. These are the address of the EEPROM of the Teensy
const int address_params = 54;

// ===== Calibrate and read Parameters =====
const double p[4] = {0.0787, -0.8471, 20.599, -22.670};
const double p_prop[3] = {128.1, -50.82, 22.06};

// ===== Proportional Control Parameters =====
const double Max_Prop = 18;
const double Min_Prop = 0;

// ===== Auto KF Parameters =====
const double max_ERR = 0.20;
const double min_ERR = -0.20;

// ===== IMU Parameters =====
const int BNO055_SAMPLERATE_DELAY_MS = 100;
const double stability_trq_gain = 1;

// ===== Torque Speed Adjust Parameters =====
const int n_step_baseline = 6;

#endif
