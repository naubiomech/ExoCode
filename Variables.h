// timer
Metro slowThisDown = Metro(1);  // Set the function to be called at no faster a rate than once per millisecond

// Variable used to schedule some actions
elapsedMillis timeElapsed;
double startTime = 0;
int streamTimerCount = 0;

// variable to indicate the beginning and ending of the stream of data
int stream = 0;

//variables used to receive and send the data from and to the gui
char holdon[24];
char *holdOnPoint = &holdon[0];
char Peek = 'a';
int cmd_from_Gui = 0;

// Single board small pin to enable the motors
const unsigned int onoff = MOTOR_ENABLE_PIN;

// Single board SQuare (big)
//const unsigned int onoff = 17;                                             //whatever the zero value is for the PID analogwrite setup
//const unsigned int which_leg_pin = WHICH_LEG_PIN;

//Includes the SoftwareSerial library to be able to use the bluetooth Serial Communication
int bluetoothTx = 0;                                                 // TX-O pin of bluetooth mate, Teensy D0
int bluetoothRx = 1;                                                 // RX-I pin of bluetooth mate, Teensy D1
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);                  // Sets an object named bluetooth to act as a serial port



bool FLAG_PRINT_TORQUES = false;
bool FLAG_PID_VALS = false;


// Variables depending on the sensor placement that are used in the state machine
bool FLAG_TWO_TOE_SENSORS = false;
bool OLD_FLAG_TWO_TOE_SENSORS = FLAG_TWO_TOE_SENSORS;

//Variables and flags for Balance control
bool FLAG_BALANCE = false;
double FLAG_BALANCE_BASELINE = 0;
double FLAG_STEADY_BALANCE_BASELINE = 0;
double count_balance = 0;
double count_steady_baseline = 0;

bool FLAG_BIOFEEDBACK = false;


// data for bluetooth autoreconnection
bool FLAG_AUTO_RECONNECT_BT = false;
bool flag_done_once_bt = false;
const unsigned int LED_BT_PIN = A11;
double LED_BT_Voltage;
int count_LED_reads;
int *p_count_LED_reads = &count_LED_reads;


// Counter msgs sent via BT
int counter_msgs = 0;


// Variables for the Control Mode
int Control_Mode = 100; // 1 for time 0 for volt 2 for proportional gain 3 for pivot proportional control
int Old_Control_Mode = Control_Mode;

//Variables to check of the motor driver error
volatile double motor_driver_count_err;
int time_err_motor;
int time_err_motor_reboot;
int flag_enable_catch_error = 1;
bool motor_error = false;


// Variables for auto KF
int flag_auto_KF = 0;

//Variables for biofeedback
volatile double Freq;
double BioFeedback_Freq_max = 1000;
double BioFeedback_Freq_min = 200;
const unsigned int pin_jack = 13;
unsigned int state = HIGH;

