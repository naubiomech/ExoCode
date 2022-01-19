//Manual Timer for Nano version
const int streamTimerCountNum = 1;
const int voltageTimerCountNum = 50;
const int32_t imuTimerCount = 50;

// Variable used to schedule some actions
elapsedMillis timeElapsed;
double startTime = 0;
int streamTimerCount = 0;
int voltageTimerCount = 0;

// variable to indicate the beginning and ending of the stream of data
int stream = 0;

//variables used to receive and send the data from and to the gui
byte holdon[96];                  //Changed to byte array
byte *holdOnPoint = &holdon[0];   //Changed to byte pointer
char Peek = 'a';
int cmd_from_Gui = 0;

// Single board small pin to enable the motors
const unsigned int onoff = MOTOR_ENABLE_PIN;

// Single board SQuare (big)

bool FLAG_PRINT_TORQUES = false;
bool FLAG_PID_VALS = false;


// Variables depending on the sensor placement that are used in the state machine
bool FLAG_ONE_TOE_SENSOR = true;
bool OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;

//Variables and flags for Balance control
bool FLAG_BALANCE = false;
double FLAG_BALANCE_BASELINE = 0;
double FLAG_STEADY_BALANCE_BASELINE = 0;
double count_balance = 0;
double count_steady_baseline = 0;

// Variables for FSR Biofeedback - added by AS 11/8/21 
bool FLAG_BIOFEEDBACK = false; 
bool AUTOADJUST_BIOFEEDBACK = false;     
const double BF_scale = 0.1; 
const double BF_upper_limit = 0.85; 
const double BF_lower_limit = 0.50;  
int biofeedback_current_success = 0; 

// data for bluetooth autoreconnection
bool FLAG_AUTO_RECONNECT_BT = false;
bool flag_done_once_bt = false;
//const unsigned int LED_BT_PIN = A11;
double LED_BT_Voltage;
int count_LED_reads;
int *p_count_LED_reads = &count_LED_reads;


// Counter msgs sent via BT
int counter_msgs = 0;

// Stim protocol activated // SS 8/6/2020
bool STIM_ACTIVATED = false;
bool Trigger_left = false;

// Variables for the Control Mode
int Control_Mode = 4; // 1 for time 0 for volt 2 for proportional gain 3 for pivot proportional control, 4 for ID PC
int Old_Control_Mode = Control_Mode;

//Variables to check of the motor driver error
volatile double motor_driver_count_err;
int time_err_motor;
int time_err_motor_reboot;
bool motor_error = false;


// Variables for auto KF
int flag_auto_KF = 1;

//Variables for biofeedback
volatile double Freq;
double BioFeedback_Freq_max = 1000;
double BioFeedback_Freq_min = 200;
const unsigned int pin_jack = 13;
unsigned int state = HIGH;
double right_stride_time, left_stride_time;
double treadmill_speed = 0.6;//subject to change
int refresh_countR = 0;
int refresh_countL = 0;

// Variables for Human-in-the-Loop Optimization (HLO)
bool Flag_HLO = false;

bool flag_motor_error_check = true;

// Flags for Proportional Control // TN 04/29/2019

bool Flag_Prop_Ctrl = false; // TN 04/29/2019
bool flag_pivot = false; // TN 04/29/2019
bool flag_id = false; // TN 04/29/2019
bool flag_resist = false; //GO 6/20/2020

// Flags for baseline communcation
bool baselineRight = false;
bool baselineLeft = false;

// Variables for Power Monitor - GO 9/24/2020

int INA219_ADR = 0x40;        // Address of INA219 for writing defined in 7 bits. The 8th bit is automatically included by Wire.read() or Wire.write()
int INA219_CONFIG = 0x00; // All-register reset, bus voltage range, PGA gain, ADC resolution/averaging. Typically does not need modification
int INA219_SHUNT = 0x01;  // Shunt voltage measurement - use this to get the shunt resistor voltage
int INA219_BUS = 0x02;    // Bus voltage measurement - use this to get the battery voltage relative to ground
int INA219_PWR = 0x03;    // Power measurement - use this to get calibrated power measurements
int INA219_CUR = 0x04;    // Current measurement - use this to get the current flowing through the shunt
int INA219_CAL = 0x05;    // Set full scale range and LSB of current/power measurements. Needed for power and current measurements
int CurrentLSB = 1;           // mA/bit. This value is used to multiply the current reading from the INA219 to obtain actual current in mA
int PowerLSB = 20 * CurrentLSB; // mW/bit. This value is used to multiply to power reading from the INA219 to obtain actual power in mW
int ShuntLSB = 0.01;          // mV. This is the default multiplier for the shunt voltage reading from the INA219.
int BusLSB = 4;               // mV. This is the multiplier for the bus (battery) voltage reading from the INA219.
int Cal = 0x5000;             // Calibration value in hex. Cal = 0.04096/(CurrentLSB*ShuntResistance). Shunt resistance on Rev3/4 is 2mOhm.

// Temporary Torque Value Storage
double temp_L_DFX = 0;
double temp_L_PFX = 0;
double temp_R_DFX = 0;
double temp_R_PFX = 0;

// Torque Offsets
double trqOffsetR = 1.219;  //DEFINE TRANSDUCER OFFSET HERE S07
double trqOffsetL = 1.503;  //DEFINE TRANSDUCER OFFSET HERE S08

// Mark Functionality
char biofeedbackLeg = 'R';
double markCount = 10;
bool markFlag = false;
