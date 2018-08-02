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
const unsigned int onoff = 22;

// Single board SQuare (big)
//const unsigned int onoff = 17;                                          //The digital pin connected to the motor on/off swich
const unsigned int zero = 2048;//1540;                                       //whatever the zero value is for the PID analogwrite setup
const unsigned int which_leg_pin = 15;

// if digital read
//const unsigned int pin_err_LL = 20;
//const unsigned int pin_err_RL = 21;
// if analog read

const unsigned int pin_err_LL = 6;
const unsigned int pin_err_RL = 7;

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
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // set pin mode for left and right sides
  pinMode(onoff, OUTPUT); //Enable disable the motors
  digitalWrite(onoff, LOW);

  //  pinMode(pin_err_LL, INPUT);
  //  pinMode(pin_err_RL, INPUT);

  pinMode(pin_err_LL, INPUT);
  pinMode(pin_err_RL, INPUT);

  pinMode(A19, INPUT); //enable the torque reading of the left torque sensor
  pinMode(A18, INPUT); //enable the torque reading of the right torque sensor

  //change the origin of the motor
  analogWrite(motor_LL_pin, zero);
  analogWrite(motor_RL_pin, zero);

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
  digitalWrite(13, HIGH);

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
volatile bool motor_error_LL = false;
volatile bool motor_error_RL = false;

volatile int Time_error_counter;
volatile int Time_error_counter_LL;
volatile int Time_error_counter_RL;

volatile double motor_driver_count_err;

double start_time_callback, start_time_timer;

int time_err_motor;
int time_err_motor_reboot;
int flag_enable_catch_error = 1;

bool motor_error = false;

void callback()//executed every 2ms
{

  //motor_error true I have an error, false I haven't
  motor_error_LL = (analogRead(pin_err_LL) <= 5);
  motor_error_RL = (analogRead(pin_err_RL) <= 5);

  motor_error = (motor_error_LL || motor_error_RL);

  if (motor_error_LL)
  {
    Time_error_counter_LL++;
  }
  else
  {
    //    Time_error_counter_LL = 0;
  }
  if (motor_error_RL)
  {
    Time_error_counter_RL++;
  }
  else
  {
    //    Time_error_counter_RL = 0;
  }

  if (stream == 1) {

    if (flag_save_EEPROM == 0) {
      flag_save_EEPROM = 1;
    }

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
  else {
    if (flag_save_EEPROM) {

//      write_FSR_values(address_FSR_LL, fsr_Left_Combined_peak_ref / 2);
//      write_FSR_values((address_FSR_LL + sizeof(double) + sizeof(char)), fsr_Left_Combined_peak_ref / 2);
//      write_FSR_values(address_FSR_RL, fsr_Right_Combined_peak_ref / 2);
//      write_FSR_values((address_FSR_RL + sizeof(double) + sizeof(char)), fsr_Right_Combined_peak_ref / 2);
//
//      write_baseline(L_baseline_address, L_p_steps->plant_peak_mean);
//      write_baseline(R_baseline_address, R_p_steps->plant_peak_mean);
      //          Serial.print("Baseline written in memory ");
      //          Serial.println(p_steps_l->plant_peak_mean);
      flag_save_EEPROM = 0;
    }
  }
  //start_time_callback = micros();

  //  start_time_timer = micros();
  //  Update_Averages();


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
    //    FSR_Average_LL = FSR_Average_LL + *(p_FSR_Array_LL + i);
    //    FSR_Average_RL = FSR_Average_RL + *(p_FSR_Array_RL + i);
    //
    //    FSR_Average_LL_Heel = FSR_Average_LL_Heel + *(p_FSR_Array_LL_Heel + i);
    //    FSR_Average_RL_Heel = FSR_Average_RL_Heel + *(p_FSR_Array_RL_Heel + i);

    if (i < dim)
    {
      Average_LL =  Average_LL + *(TarrayPoint_LL + i);
      Average_RL =  Average_RL + *(TarrayPoint_RL + i);
      //        Average_RL =  Average_RL + *(TarrayPoint_RL + i);
    }
  }

  //  Average_Volt_LL = FSR_Average_LL / dim_FSR;
  //  Average_Volt_RL = FSR_Average_RL / dim_FSR;
  //
  //  Average_Volt_LL_Heel = FSR_Average_LL_Heel / dim_FSR;
  //  Average_Volt_RL_Heel = FSR_Average_RL_Heel / dim_FSR;
  //
  //  Average_Trq_LL = Average_LL / dim;
  //  Average_Trq_RL = Average_RL / dim;
  //
  //  Combined_Average_LL = (FSR_Average_LL + FSR_Average_LL_Heel) / dim_FSR;
  //  Combined_Average_RL = (FSR_Average_RL + FSR_Average_RL_Heel) / dim_FSR;

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

  //--------------------------------
  //  Serial.println();
  //  Serial.print("Time averages: ");
  //  Serial.println(micros() - start_time_timer);
  //  start_time_timer = micros();
  if (FSR_CAL_FLAG) {
    //    Serial.println("Going to calib");

    FSR_calibration();
    //    FSR_baseline_FLAG = 1;
  }

  //  if (FSR_baseline_FLAG_Left) {
  //
  //    base_1 = take_baseline(L_state, L_state_old, L_p_steps);
  //
  //    if (base_1) {
  //      Serial.println("Left Baseline token");
  //      FSR_baseline_FLAG_Left = 0;
  //    }
  //  }

  if (FSR_baseline_FLAG_Right) {
    take_baseline(R_state, R_state_old, R_p_steps, p_FSR_baseline_FLAG_Right, R_baseline_address);
  }
  if (FSR_baseline_FLAG_Left) {
    take_baseline(L_state, L_state_old, L_p_steps, p_FSR_baseline_FLAG_Left, L_baseline_address);
  }

  //
  //
  if (stream == 1)
  {
    if (streamTimerCount >= 5)
    {
      //      start_time_timer = micros();
      send_data_message_wc();
      streamTimerCount = 0;
      //      Serial.print("Time sending data : ");
      //      Serial.println(micros() - start_time_timer);
      //      if (flag_13) {
      //        if (flag_count > 10) {
      //          digitalWrite(13, HIGH);
      //          flag_13 = 0;
      //          flag_count = 0;
      //        }
      //        else {
      //          flag_count++;
      //        }
      //      } else {
      //        if (flag_count > 10) {
      //          digitalWrite(13, LOW);
      //          flag_13 = 1;
      //          flag_count = 0;
      //        }
      //        else {
      //          flag_count++;
      //        }
      //      }
    }



    if (streamTimerCount == 1 && flag_auto_KF == 1)
      Auto_KF();

    streamTimerCount++;

    //Apply PID control
    //    start_time_timer = micros();

    pid(Average_Trq_LL, 1);
    pid(Average_Trq_RL, 2);

    //    Serial.print("Time PIDs : ");
    //    Serial.println(micros() - start_time_timer);
    //    start_time_timer = micros();

    state_machine_LL();  //for LL
    state_machine_RL();  //for RL

    //    Serial.print("Time State Machines : ");
    //    Serial.println(micros() - start_time_timer);
    //    start_time_timer = micros();

    set_2_zero_if_steady_state();

    N3_LL = Ctrl_ADJ(L_state, L_state_old, L_p_steps, N3_LL, New_PID_Setpoint_LL, p_Setpoint_Ankle_LL, p_Setpoint_Ankle_LL_Pctrl, Trq_time_volt, L_Prop_Gain, FSR_baseline_FLAG_Left, p_L_FSR_Ratio, p_L_Max_FSR_Ratio);
    if (Trq_time_volt == 1) {

      if (L_state == 3) {

                  if ((abs(New_PID_Setpoint_LL - *p_Setpoint_Ankle_LL) > 0.1))// && (sigm_done_LL))
        //            //          Previous_Setpoint_Ankle_LL = New_PID_Setpoint_LL;
        //            if (abs(*p_Setpoint_Ankle_LL > New_PID_Setpoint_LL)) {
        //            Old_PID_Setpoint_LL = New_PID_Setpoint_LL;
        New_PID_Setpoint_LL = *p_Setpoint_Ankle_LL;
        //        }
      }
      //      if ((n_iter_LL >= N_step_LL) && (abs(New_PID_Setpoint_LL - PID_Setpoint_LL) <= 0.1))
      //      {
      //        sigm_done_LL = true;
      //      }
      else {
        New_PID_Setpoint_LL = 0;
        *p_L_Max_FSR_Ratio = 0;
      }
    }


    N3_RL = Ctrl_ADJ(R_state, R_state_old, R_p_steps, N3_RL, New_PID_Setpoint_RL, p_Setpoint_Ankle_RL, p_Setpoint_Ankle_RL_Pctrl, Trq_time_volt, R_Prop_Gain, FSR_baseline_FLAG_Right, p_R_FSR_Ratio, p_R_Max_FSR_Ratio);
    //    if (Trq_time_volt == 1) New_PID_Setpoint_RL = *p_Setpoint_Ankle_RL;
    //    Serial.println(micros() - start_time_timer);
  }

  //  if (micros() - start_time_callback >= 600) {
  //    Serial.println( micros() - start_time_callback);
  //  }
  //Serial.println(analogRead(fsr_sense_Left_Heel)+analogRead(fsr_sense_Left_Toe));
  //Serial.println(analogRead(fsr_sense_Right_Heel)+analogRead(fsr_sense_Right_Toe));
}// end callback
//
//const unsigned int fsr_sense_Left_Heel = A14;
//const unsigned int fsr_sense_Left_Toe = A15;
//const unsigned int fsr_sense_Right_Heel = A12;
//const unsigned int fsr_sense_Right_Toe = A13;
void loop()
{

  if (slowThisDown.check() == 1) // If the time passed is over 1ms is a true statement
  {
    //    start_time_timer = millis();
    if (bluetooth.available() > 0)
    {
      receive_and_transmit();       //Recieve and transmit was moved here so it will not interfere with the data message
    }

    //    if (digitalRead(pin_err_LL) == HIGH || digitalRead(pin_err_RL) == HIGH) {
    //      if (time_err_motor == 0)
    //        digitalWrite(onoff, LOW);
    //
    //      if (time_err_motor >= 200) {
    //        digitalWrite(onoff, HIGH);
    //      }
    //
    //      motor_driver_count_err++;
    //      time_err_motor++;
    //
    //      if (time_err_motor >= 300) {
    //        time_err_motor = 0;
    //      }


    //      delayMicroseconds(1000);
    //      digitalWrite(onoff, HIGH);

    //    }

    slowThisDown.reset();     //Resets the interval
    //    if (millis() - start_time_timer >= 1) {
    //      Serial.print("Time timer : ");
    //      Serial.println( millis() - start_time_callback);
    //    }
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

    //New at 06/05/2018
    Previous_Setpoint_Ankle_LL = 0;
    Previous_Setpoint_Ankle_RL = 0;
    R_coef_in_3_steps = 0;
    R_num_3_steps = 0;
    L_coef_in_3_steps = 0;
    L_num_3_steps = 0;

    R_1st_step = 1;
    L_1st_step = 1;
    //New at 06/05/2018



    Time_error_counter_LL = 0;
    Time_error_counter_RL = 0;


  }// End else


}
