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

#define IMU_BOARD

#include "Board.h"
#include "Leg.h"
#include <elapsedMillis.h>
#include <EEPROM.h>
#include "TimerOne.h"
#include <PID_v2.h>
#include <SoftwareSerial.h>
// #include "Torque_Speed_ADJ.h"
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
#include "IMU.h"

Metro slowThisDown = Metro(1);  // Set the function to be called at no faster a rate than once per millisecond
Metro BnoControl = Metro(10);

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

//Includes the SoftwareSerial library to be able to use the bluetooth Serial Communication
int bluetoothTx = 0;                                                 // TX-O pin of bluetooth mate, Teensy D0
int bluetoothRx = 1;                                                 // RX-I pin of bluetooth mate, Teensy D1
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);                  // Sets an object named bluetooth to act as a serial port

void setup()
{

  // enable bluetooth
  bluetooth.begin(115200);
  Serial.begin(115200);

  Serial.println("Starting");

  if (!bno.begin())
  {
    Serial.println("No IMU detected haulting...");
    while (1);
  }
  Serial.println("IMU setup");


  initialize_left_leg(left_leg);
  initialize_right_leg(right_leg);

  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  // The led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // set pin mode for left and right sides
  pinMode(onoff, OUTPUT); //Enable disable the motors
  digitalWrite(onoff, LOW);

  //  pinMode(left_leg->pin_err, INPUT);
  //  pinMode(right_leg->pin_err, INPUT);

  pinMode(left_leg->pin_err, INPUT);
  pinMode(right_leg->pin_err, INPUT);

  pinMode(TORQUE_SENSOR_LEFT_ANKLE_PIN, INPUT); //enable the torque reading of the left torque sensor
  pinMode(TORQUE_SENSOR_RIGHT_ANKLE_PIN, INPUT); //enable the torque reading of the right torque sensor

  //change the origin of the motor
  analogWrite(MOTOR_LEFT_ANKLE_PIN, zero);
  analogWrite(MOTOR_RIGHT_ANKLE_PIN, zero);

  //  Left PID
  left_leg->pid.SetMode(AUTOMATIC);
  left_leg->pid.SetTunings(left_leg->kp, left_leg->ki, left_leg->kd);                                      //Kp, Ki, Kd ##COULD BE AUTOTUNED
  left_leg->pid.SetOutputLimits(-1500, 1500);                                  //range of Output around 0 ~ 1995 ##THIS IS DIFFERENT NOW AND SHOULD CONCRETELY CONFIRM
  left_leg->pid.SetSampleTime(PID_sample_time);                              //what is the sample time we want in millis

  //  Right PID
  right_leg->pid.SetMode(AUTOMATIC);
  right_leg->pid.SetTunings(right_leg->kp, right_leg->ki, right_leg->kd);                                      //Kp, Ki, Kd ##COULD BE AUTOTUNED
  right_leg->pid.SetOutputLimits(-1500, 1500);                                  //range of Output around 0 ~ 1995 ##THIS IS DIFFERENT NOW AND SHOULD CONCRETELY CONFIRM
  right_leg->pid.SetSampleTime(PID_sample_time);                              //what is the sample time we want in millis

  // Fast torque calibration
  torque_calibration();

  left_leg->p_steps->fsr_Toe = left_leg->fsr_sense_Toe;
  right_leg->p_steps->fsr_Toe = right_leg->fsr_sense_Toe;


  //  left_leg->p_FSR_Array = &left_leg->FSR_Average_array[0];
  //  right_leg->p_FSR_Array = &right_leg->FSR_Average_array[0];
  digitalWrite(LED_PIN, HIGH);

  // set the interrupt
  Timer1.initialize(2000);         // initialize timer1, and set a 10 ms period *note this is 10k microseconds*
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}


int Trq_time_volt = 0; // 1 for time 0 for volt 2 for proportional gain 3 for pivot proportional control
int Old_Trq_time_volt = Trq_time_volt;
int flag_13 = 1;
int flag_count = 0;

int flag_semaphore = 0;

volatile double motor_driver_count_err;

double start_time_callback, start_time_timer;

int time_err_motor;
int time_err_motor_reboot;
int flag_enable_catch_error = 1;

bool motor_error = true;

void callback()//executed every 2ms
{
  
  resetMotorIfError();

  calculate_averages();

  check_FSR_calibration();

  rotate_motor();

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

  if (BnoControl.check()) {
    Serial.println("Vector: ");
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Serial.print("X: ");
    Serial.print(euler.x(), 4);
    Serial.print("\tY: ");
    Serial.print(euler.y(), 4);
    Serial.print("\tZ: ");
    Serial.print(euler.z(), 4);
    Serial.println();
  }

  if (stream != 1)
  {
    reset_starting_parameters();
  }// End else
}

void resetMotorIfError() {
  //motor_error true I have an error, false I haven't

  motor_error = ((analogRead(left_leg->pin_err) <= 5) || (analogRead(right_leg->pin_err) <= 5));

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

void calculate_averages() {
  //Calc the average value of Torque

  //Shift the arrays
  for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
    *(left_leg->TarrayPoint + j) = *(left_leg->TarrayPoint + j - 1);                //Puts the element in the following memory space into the current memory space
    *(right_leg->TarrayPoint + j) = *(right_leg->TarrayPoint + j - 1);
  }

  //Get the torques
  *(left_leg->TarrayPoint) = get_LL_torq();
  *(right_leg->TarrayPoint) = get_RL_torq();

  //  noInterrupts();
  left_leg->FSR_Average = 0;
  right_leg->FSR_Average = 0;
  left_leg->FSR_Average_Heel = 0;
  right_leg->FSR_Average_Heel = 0;
  left_leg->Average = 0;
  right_leg->Average = 0;

  for (int i = 0; i < dim_FSR; i++)
  {

    if (i < dim)
    {
      left_leg->Average =  left_leg->Average + *(left_leg->TarrayPoint + i);
      right_leg->Average =  right_leg->Average + *(right_leg->TarrayPoint + i);
      //        right_leg->Average =  right_leg->Average + *(right_leg->TarrayPoint + i);
    }
  }

  // if not filtering the FSR anymore
  left_leg->FSR_Average = fsr(left_leg->fsr_sense_Toe);
  left_leg->Average_Volt = left_leg->FSR_Average;

  left_leg->FSR_Average_Heel = fsr(left_leg->fsr_sense_Heel);
  left_leg->Average_Volt_Heel = left_leg->FSR_Average_Heel;

  right_leg->FSR_Average = fsr(right_leg->fsr_sense_Toe);
  right_leg->Average_Volt = right_leg->FSR_Average;

  right_leg->FSR_Average_Heel = fsr(right_leg->fsr_sense_Heel);
  right_leg->Average_Volt_Heel = right_leg->FSR_Average_Heel;

  left_leg->Combined_Average = (left_leg->FSR_Average + left_leg->FSR_Average_Heel);
  right_leg->Combined_Average = (right_leg->FSR_Average + right_leg->FSR_Average_Heel);

  left_leg->Average_Trq = left_leg->Average / dim;
  right_leg->Average_Trq = right_leg->Average / dim;

  left_leg->p_steps->curr_voltage = left_leg->Combined_Average;
  right_leg->p_steps->curr_voltage = right_leg->Combined_Average;

  left_leg->p_steps->torque_average = left_leg->Average / dim;
  right_leg->p_steps->torque_average = right_leg->Average / dim;

}

void check_FSR_calibration() {

  if (FSR_CAL_FLAG) {
    FSR_calibration();
  }

  if (right_leg->FSR_baseline_FLAG) {
    take_baseline(right_leg->state, right_leg->state_old, right_leg->p_steps, right_leg->p_FSR_baseline_FLAG);
  }
  if (left_leg->FSR_baseline_FLAG) {
    take_baseline(left_leg->state, left_leg->state_old, left_leg->p_steps, left_leg->p_FSR_baseline_FLAG);
  }

}

void rotate_motor() {
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

    pid(left_leg->Average_Trq, 1);
    pid(right_leg->Average_Trq, 2);

    state_machine_LL();  //for LL
    state_machine_RL();  //for RL

    set_2_zero_if_steady_state();

    left_leg->N3 = Ctrl_ADJ(left_leg->state, left_leg->state_old, left_leg->p_steps, left_leg->N3, left_leg->New_PID_Setpoint, left_leg->p_Setpoint_Ankle, left_leg->p_Setpoint_Ankle_Pctrl, Trq_time_volt, left_leg->Prop_Gain, left_leg->FSR_baseline_FLAG);
    right_leg->N3 = Ctrl_ADJ(right_leg->state, right_leg->state_old, right_leg->p_steps, right_leg->N3, right_leg->New_PID_Setpoint, right_leg->p_Setpoint_Ankle, right_leg->p_Setpoint_Ankle_Pctrl, Trq_time_volt, right_leg->Prop_Gain, right_leg->FSR_baseline_FLAG);
  }
}

void reset_starting_parameters() {
  //Reset the starting values
  left_leg->p_steps->count_plant = 0;
  left_leg->p_steps->n_steps = 0;
  left_leg->p_steps->flag_start_plant = false;
  left_leg->p_steps->flag_take_average = false;
  left_leg->p_steps->flag_N3_adjustment_time = false;
  left_leg->p_steps->flag_take_baseline = false;
  left_leg->p_steps->torque_adj = false;

  right_leg->p_steps->count_plant = 0;
  right_leg->p_steps->n_steps = 0;
  right_leg->p_steps->flag_start_plant = false;
  right_leg->p_steps->flag_take_average = false;
  right_leg->p_steps->flag_N3_adjustment_time = false;
  right_leg->p_steps->flag_take_baseline = false;
  right_leg->p_steps->torque_adj = false;

  left_leg->N3 = N3;
  left_leg->N2 = N2;
  left_leg->N1 = N1;

  right_leg->N3 = N3;
  right_leg->N2 = N2;
  right_leg->N1 = N1;

  left_leg->p_steps->perc_l = 0.5;
  right_leg->p_steps->perc_l = 0.5;

  left_leg->activate_in_3_steps = 1;
  right_leg->activate_in_3_steps = 1;

  left_leg->Previous_Setpoint_Ankle = 0;
  right_leg->Previous_Setpoint_Ankle = 0;
  right_leg->coef_in_3_steps = 0;
  right_leg->num_3_steps = 0;
  left_leg->coef_in_3_steps = 0;
  left_leg->num_3_steps = 0;

  right_leg->first_step = 1;
  left_leg->first_step = 1;


}

