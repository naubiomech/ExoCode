// This is the code for the Single board Ankle Exoskeleton -> A_EXO_s
//
// FSR sensors retrieve the sensor voltage related to the foot pressure.
// The state machine (Left and Right state machine) identify the participant status depending on the voltage.
// The torque reference is decided by the user (Matlab GUI) and send as reference to the PID control.
// The PID control and data tranmission to the GUI are scheduled by an interrupt.
//
// Sensor can be calibrated by the user and/or saved calibration can be loaded to speed up the setup.
//
// The torque reference can be smoothed by sigmoid functions
// In case of too long steady state the torque reference is set to zero
// In case of new torque reference the torque amount is provided gradually as function of the steps.
//
// Ex: Torque ref = 10N
// 1 step = 0N
// 2 steps = 2N
// 3 steps = 4N
// 4 steps = 6N
// 5 steps = 8N
// 6 steps = 10N
//
// The torque and the shaping function can be adapted as function of the force pressure/Voltage
// and averaged speed/step duration
//
// Several parameters can be modified thanks to the Receive and Transmit functions

#define TWO_LEG_BOARD

const int dim_FSR = 30;
double FSR_Average_RL_array[dim_FSR] = {0};
double * p_FSR_Array_RL = &FSR_Average_RL_array[0];
double FSR_Average_RL = 0;
double Curr_FSR_RL = 0;

double FSR_Average_LL_array[dim_FSR] = {0};
double * p_FSR_Array_LL = &FSR_Average_LL_array[0];
double FSR_Average_LL = 0;
double Curr_FSR_LL = 0;

//const int dim_FSR = 30;
double FSR_Average_RL_array_Heel[dim_FSR] = {0};
double * p_FSR_Array_RL_Heel = &FSR_Average_RL_array_Heel[0];
double FSR_Average_RL_Heel = 0;
double Curr_FSR_RL_Heel = 0;

double FSR_Average_LL_array_Heel[dim_FSR] = {0};
double * p_FSR_Array_LL_Heel = &FSR_Average_LL_array_Heel[0];
double FSR_Average_LL_Heel = 0;
double Curr_FSR_LL_Heel = 0;

const int dim = 5;
double Tarray_LL[dim] = {0};
double * TarrayPoint_LL = &Tarray_LL[0];
double Average_LL = 0;

double Tarray_RL[dim] = {0};
double * TarrayPoint_RL = &Tarray_RL[0];
double Average_RL = 0;

double R_sign = 1;
double L_sign = 1;

#include "Board.h"
#include <elapsedMillis.h>
#include <EEPROM.h>
#include "TimerOne.h"
#include <PID_v2.h>
#include <SoftwareSerial.h>
#include "Torque_Speed_ADJ.h"
#include "Memory_address.h"
#include "Shaping_Parameters.h"
#include "FSR_Parameters.h"
#include "PID_and_Ctrl_Parameters.h"
#include "Filter_Parameters.h"
#include "State_Machine_Parameters.h"
#include "Reference_ADJ.h"
#include "Msg_functions.h"
#include "Calibrate_and_Read_Sensors.h"
#include "Proportional_Ctrl.h"
#include "Auto_KF.h"
#include "Combined_FSR.h"
#include <Metro.h> // Include the Metro library


Metro slowThisDown = Metro(1);  // Set the function to be called at no faster a rate than once per millisecond

//To interrupt and to schedule we take advantage of the
elapsedMillis timeElapsed;
double startTime = 0;
int streamTimerCount = 0;

int stream = 0;

char holdon[24];
char *holdOnPoint = &holdon[0];
char Peek = 'a';
int cmd_from_Gui = 0;

// Single board small
const unsigned int onoff = MOTOR_ENABLE_PIN;

// Single board SQuare (big)
//const unsigned int onoff = 17;                                          //The digital pin connected to the motor on/off swich
const unsigned int zero = 2048;//1540;                                       //whatever the zero value is for the PID analogwrite setup
const unsigned int which_leg_pin = WHICH_LEG_PIN;

// if digital read
//const unsigned int pin_err_LL = 20;
//const unsigned int pin_err_RL = 21;
// if analog read

const unsigned int pin_err_LL = MOTOR_ERROR_LEFT_ANKLE_PIN;
const unsigned int pin_err_RL = MOTOR_ERROR_RIGHT_ANKLE_PIN;

//Includes the SoftwareSerial library to be able to use the bluetooth Serial Communication
int bluetoothTx = 0;                                                 // TX-O pin of bluetooth mate, Teensy D0
int bluetoothRx = 1;                                                 // RX-I pin of bluetooth mate, Teensy D1
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);                  // Sets an object named bluetooth to act as a serial port

double store_KF_LL = 0;
double store_KF_RL = 0;

void setup()
{



  // set the interrupt
  Timer1.initialize(2000);         // initialize timer1, and set a 10 ms period *note this is 10k microseconds*
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  //  Timer2.initialize(2000);

  //  Timer2.initialize(2000);

  // enable bluetooth
  bluetooth.begin(115200);
  Serial.begin(115200);
  //while (!Serial) {};

  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  // The led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // set pin mode for left and right sides
  pinMode(onoff, OUTPUT); //Enable disable the motors
  digitalWrite(onoff, LOW);

  //  pinMode(pin_err_LL, INPUT);
  //  pinMode(pin_err_RL, INPUT);

  pinMode(pin_err_LL, INPUT);
  pinMode(pin_err_RL, INPUT);

  pinMode(TORQUE_SENSOR_LEFT_ANKLE_PIN, INPUT); //enable the torque reading of the left torque sensor
  pinMode(TORQUE_SENSOR_RIGHT_ANKLE_PIN, INPUT); //enable the torque reading of the right torque sensor

  //change the origin of the motor
  analogWrite(MOTOR_LEFT_ANKLE_PIN, zero);
  analogWrite(MOTOR_RIGHT_ANKLE_PIN, zero);

  //  Left PID
  PID_LL.SetMode(AUTOMATIC);
  PID_LL.SetTunings(kp_LL, ki_LL, kd_LL);                                      //Kp, Ki, Kd ##COULD BE AUTOTUNED
  PID_LL.SetOutputLimits(-1500, 1500);                                  //range of Output around 0 ~ 1995 ##THIS IS DIFFERENT NOW AND SHOULD CONCRETELY CONFIRM
  PID_LL.SetSampleTime(PID_sample_time);                              //what is the sample time we want in millis

  //  Right PID
  PID_RL.SetMode(AUTOMATIC);
  PID_RL.SetTunings(kp_RL, ki_RL, kd_RL);                                      //Kp, Ki, Kd ##COULD BE AUTOTUNED
  PID_RL.SetOutputLimits(-1500, 1500);                                  //range of Output around 0 ~ 1995 ##THIS IS DIFFERENT NOW AND SHOULD CONCRETELY CONFIRM
  PID_RL.SetSampleTime(PID_sample_time);                              //what is the sample time we want in millis

  // Fast torque calibration
  torque_calibration();

  L_p_steps->fsr_Toe = fsr_sense_Left_Toe;
  R_p_steps->fsr_Toe = fsr_sense_Right_Toe;


  //  p_FSR_Array_LL = &FSR_Average_LL_array[0];
  //  p_FSR_Array_RL = &FSR_Average_RL_array[0];
  digitalWrite(LED_PIN, HIGH);

}


int Trq_time_volt = 0; // 1 for time 0 for volt 2 for proportional gain 3 for pivot proportional control
int Old_Trq_time_volt = Trq_time_volt;
int flag_13 = 1;
int flag_count = 0;

int flag_semaphore = 0;


double previous_curr_voltage_LL ;
double previous_curr_voltage_RL ;
double previous_torque_average_LL ;
double previous_torque_average_RL ;

volatile double Average_Volt_LL;
volatile double Average_Volt_RL;
volatile double Average_Volt_LL_Heel;
volatile double Average_Volt_RL_Heel;
volatile double Average_Trq_LL;
volatile double Average_Trq_RL;
volatile double Combined_Average_LL;
volatile double Combined_Average_RL;

volatile double motor_driver_count_err;

double start_time_callback, start_time_timer;

int time_err_motor;
int time_err_motor_reboot;
int flag_enable_catch_error = 1;

bool motor_error = true;

void callback()//executed every 2ms
{

  resetMotorIfError();

  //Calc the average value of Torque

  //Shift the arrays
  for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
    *(TarrayPoint_LL + j) = *(TarrayPoint_LL + j - 1);                //Puts the element in the following memory space into the current memory space
    *(TarrayPoint_RL + j) = *(TarrayPoint_RL + j - 1);
  }

  //Get the torques
  *(TarrayPoint_LL) = get_LL_torq();
  *(TarrayPoint_RL) = get_RL_torq();

  //  noInterrupts();
  FSR_Average_LL = 0;
  FSR_Average_RL = 0;
  FSR_Average_LL_Heel = 0;
  FSR_Average_RL_Heel = 0;
  Average_LL = 0;
  Average_RL = 0;

  for (int i = 0; i < dim_FSR; i++)
  {

    if (i < dim)
    {
      Average_LL =  Average_LL + *(TarrayPoint_LL + i);
      Average_RL =  Average_RL + *(TarrayPoint_RL + i);
      //        Average_RL =  Average_RL + *(TarrayPoint_RL + i);
    }
  }

  // if not filtering the FSR anymore
  FSR_Average_LL = fsr(fsr_sense_Left_Toe);
  Average_Volt_LL = FSR_Average_LL;

  FSR_Average_LL_Heel = fsr(fsr_sense_Left_Heel);
  Average_Volt_LL_Heel = FSR_Average_LL_Heel;

  FSR_Average_RL = fsr(fsr_sense_Right_Toe);
  Average_Volt_RL = FSR_Average_RL;

  FSR_Average_RL_Heel = fsr(fsr_sense_Right_Heel);
  Average_Volt_RL_Heel = FSR_Average_RL_Heel;

  Combined_Average_LL = (FSR_Average_LL + FSR_Average_LL_Heel);
  Combined_Average_RL = (FSR_Average_RL + FSR_Average_RL_Heel);

  Average_Trq_LL = Average_LL / dim;
  Average_Trq_RL = Average_RL / dim;

  L_p_steps->curr_voltage = Combined_Average_LL;
  R_p_steps->curr_voltage = Combined_Average_RL;

  L_p_steps->torque_average = Average_LL / dim;
  R_p_steps->torque_average = Average_RL / dim;

  if (FSR_CAL_FLAG) {

    FSR_calibration();

  }

  if (FSR_baseline_FLAG_Right) {
    take_baseline(R_state, R_state_old, R_p_steps, p_FSR_baseline_FLAG_Right);
  }
  if (FSR_baseline_FLAG_Left) {
    take_baseline(L_state, L_state_old, L_p_steps, p_FSR_baseline_FLAG_Left);
  }

  if (stream == 1)
  {
    if (streamTimerCount >= 5)
    {
      send_data_message_wc();
      streamTimerCount = 0;
    }

    if (streamTimerCount == 1 && flag_auto_KF == 1)
      Auto_KF();

    streamTimerCount++;

    pid(Average_Trq_LL, 1);
    pid(Average_Trq_RL, 2);

    state_machine_LL();  //for LL
    state_machine_RL();  //for RL

    set_2_zero_if_steady_state();

    N3_LL = Ctrl_ADJ(L_state, L_state_old, L_p_steps, N3_LL, New_PID_Setpoint_LL, p_Setpoint_Ankle_LL, p_Setpoint_Ankle_LL_Pctrl, Trq_time_volt, L_Prop_Gain, FSR_baseline_FLAG_Left);
    N3_RL = Ctrl_ADJ(R_state, R_state_old, R_p_steps, N3_RL, New_PID_Setpoint_RL, p_Setpoint_Ankle_RL, p_Setpoint_Ankle_RL_Pctrl, Trq_time_volt, R_Prop_Gain, FSR_baseline_FLAG_Right);
  }

}// end callback

void loop()
{

  if (slowThisDown.check() == 1) // If the time passed is over 1ms is a true statement
  {
    //    start_time_timer = millis();
    if (bluetooth.available() > 0)
    {
      receive_and_transmit();       //Recieve and transmit was moved here so it will not interfere with the data message
    }

    slowThisDown.reset();     //Resets the interval
  }



  if (stream == 1)
  {
  }
  else
  {
    //Reset the starting values
    L_p_steps->count_plant = 0;
    L_p_steps->n_steps = 0;
    L_p_steps->flag_start_plant = false;
    L_p_steps->flag_take_average = false;
    L_p_steps->flag_N3_adjustment_time = false;
    L_p_steps->flag_take_baseline = false;
    L_p_steps->torque_adj = false;

    R_p_steps->count_plant = 0;
    R_p_steps->n_steps = 0;
    R_p_steps->flag_start_plant = false;
    R_p_steps->flag_take_average = false;
    R_p_steps->flag_N3_adjustment_time = false;
    R_p_steps->flag_take_baseline = false;
    R_p_steps->torque_adj = false;

    N3_LL = N3;
    N2_LL = N2;
    N1_LL = N1;

    N3_RL = N3;
    N2_RL = N2;
    N1_RL = N1;

    L_p_steps->perc_l = 0.5;
    R_p_steps->perc_l = 0.5;

    L_activate_in_3_steps = 1;
    R_activate_in_3_steps = 1;

    Previous_Setpoint_Ankle_LL = 0;
    Previous_Setpoint_Ankle_RL = 0;
    R_coef_in_3_steps = 0;
    R_num_3_steps = 0;
    L_coef_in_3_steps = 0;
    L_num_3_steps = 0;

    R_1st_step = 1;
    L_1st_step = 1;

  }// End else
}

void resetMotorIfError(){
  //motor_error true I have an error, false I haven't

  motor_error = ((analogRead(pin_err_LL) <= 5) || (analogRead(pin_err_RL) <= 5));

  if (stream == 1) {

    if (not(motor_error) && (digitalRead(onoff) == LOW)) {
      digitalWrite(onoff, HIGH);
    }

    if (motor_error && (flag_enable_catch_error == 0)) {
      flag_enable_catch_error = 1;
    }

    if (flag_enable_catch_error) {
      if (time_err_motor == 0) {
        digitalWrite(onoff, LOW);
        time_err_motor_reboot = 0;
      }

      motor_driver_count_err++;
      time_err_motor++;

      //was time_err_motor >= 4
      if (time_err_motor >= 8) {
        digitalWrite(onoff, HIGH);
        time_err_motor_reboot++;
        if (time_err_motor_reboot >= 12) {
          flag_enable_catch_error = 0;
          time_err_motor = 0;
        }
      }

    }// end if flag_enable_catch_error==1;

  }//end stream==1
}

