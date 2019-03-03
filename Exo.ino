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
// and averaged speed/step duration. See Control_Mode in Control_Adjustment
//
// Several parameters can be modified thanks to the Receive and Transmit functions

#define TWO_LEG_BOARD
//The digital pin connected to the motor on/off swich
const unsigned int zero = 2048;//1540;

#include "Parameters.h"
#include "Board.h"
#include "Leg.h"
#include <elapsedMillis.h>
#include <EEPROM.h>
#include "TimerOne.h"
#include <PID_v2.h>
#include <SoftwareSerial.h>
//#include <NewSoftSerial.h>
#include "Reference_ADJ.h"
#include "Msg_functions.h"
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
//const unsigned int onoff = 17;                                             //whatever the zero value is for the PID analogwrite setup
const unsigned int which_leg_pin = WHICH_LEG_PIN;

//Includes the SoftwareSerial library to be able to use the bluetooth Serial Communication
int bluetoothTx = 0;                                                 // TX-O pin of bluetooth mate, Teensy D0
int bluetoothRx = 1;                                                 // RX-I pin of bluetooth mate, Teensy D1
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);                  // Sets an object named bluetooth to act as a serial port



bool FLAG_PRINT_TORQUES = false;
bool FLAG_PID_VALS = false;


// Sensor placement
bool FLAG_TWO_TOE_SENSORS = true;
bool OLD_FLAG_TWO_TOE_SENSORS = FLAG_TWO_TOE_SENSORS;

//Variables and flags for Balance control
bool FLAG_BALANCE = false;
double FLAG_BALANCE_BASELINE = 0;
double FLAG_STEADY_BALANCE_BASELINE = 0;
double count_balance = 0;
double count_steady_baseline = 0;

bool FLAG_BIOFEEDBACK = false;


// BT autoreconnection
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

//Variables for the check of the motor driver error
volatile double motor_driver_count_err;
int time_err_motor;
int time_err_motor_reboot;
int flag_enable_catch_error = 1;
bool motor_error = false;


// Variables for auto KF
int flag_auto_KF = 0;

volatile double Freq;
double BioFeedback_Freq_max = 1000;
double BioFeedback_Freq_min = 200;
const unsigned int pin_jack = 13;
unsigned int state = HIGH;


//----------------------------------------------------------------------------------


// Initialize the system
void setup()
{
  // set the interrupt
  Timer1.initialize(2000);         // initialize timer1, and set a 10 ms period *note this is 10k microseconds*
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  // enable bluetooth
  bluetooth.begin(115200);
  Serial.begin(115200);

  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  initialize_left_leg(left_leg);
  initialize_right_leg(right_leg);

  torque_calibration();

  // The led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // set pin mode for left and right sides
  pinMode(onoff, OUTPUT); //Enable disable the motors
  digitalWrite(onoff, LOW);

  // Fast torque calibration
  torque_calibration();

  digitalWrite(LED_PIN, HIGH);

}

//----------------------------------------------------------------------------------

void callback()//executed every 2ms
{

  resetMotorIfError();

  calculate_averages();

  check_FSR_calibration();

  rotate_motor();

  check_Balance_Baseline();

  if (left_leg->BIO_BASELINE_FLAG) {
    BioFeedback_Baseline(left_leg);//just left leg for now
  }

  if (FLAG_AUTO_RECONNECT_BT) {
    LED_BT_Voltage = Check_LED_BT(LED_BT_PIN, LED_BT_Voltage, p_count_LED_reads);
  }

  if (FLAG_BIOFEEDBACK) {
    if (left_leg->Frequency >= right_leg->Frequency) {
      Freq = left_leg->Frequency;
    } else {
      Freq = right_leg->Frequency;
    }

  }

}// end callback

//----------------------------------------------------------------------------------

void loop()
{

  if (slowThisDown.check() == 1) // If the time passed is over 1ms is a true statement
  {
    if (bluetooth.available() > 0)
    {
      receive_and_transmit();       //Recieve and transmit was moved here so it will not interfere with the data message
    }

    slowThisDown.reset();     //Resets the interval
  }


  biofeedback();


  if (stream != 1)
  {
    reset_starting_parameters();
    flag_done_once_bt = false;
  }// End else
}// end void loop
//---------------------------------------------------------------------------------

void biofeedback() {

  if (left_leg->NO_Biofeedback || left_leg->BioFeedback_Baseline_flag == false || FLAG_BIOFEEDBACK == false) {
  } else {
    //    Serial.println((left_leg->start_time_Biofeedback) - millis());
    if (abs(left_leg->start_time_Biofeedback - millis()) >= Freq) {
      Serial.println((left_leg->start_time_Biofeedback) - millis());
      state = digitalRead(LED_PIN);
      if (state == HIGH) {
        state = LOW;
      } else {
        state = HIGH;
      }
      digitalWrite(LED_PIN, state);
      //        tone(pin_jack, 500, 100);
      left_leg->start_time_Biofeedback = millis();

//      analogWrite(A17, 5);
      tone(A17, 500, 100);




      //      if(state==5){state=0;}else{state=5;}
      Serial.print(" Freq : ");
      Serial.println(Freq);

      //    } else {
      //       analogWrite(pin_jack, LOW);
    }
  }
  return;
}

//----------------------------------------------------------------------------------

void resetMotorIfError() {
  //motor_error true I have an error, false I haven't
  left_leg->motor_error = (analogRead(left_leg->pin_err) <= 5);
  right_leg->motor_error = (analogRead(right_leg->pin_err) <= 5);

  motor_error = (left_leg->motor_error || right_leg->motor_error);

  if (left_leg->motor_error) {
    left_leg->Time_error_counter++;
  }

  if (right_leg->motor_error) {
    right_leg->Time_error_counter++;
  }


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

//----------------------------------------------------------------------------------

void calculate_leg_average(Leg* leg) {
  //Calc the average value of Torque

  //Shift the arrays
  for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
    leg->TarrayPoint[j] = leg->TarrayPoint[j - 1];                //Puts the element in the following memory space into the current memory space
  }
  //Get the torque
  leg->TarrayPoint[0] = get_torq(leg);
  leg->FSR_Toe_Average = 0;
  leg->FSR_Heel_Average = 0;
  leg->Average = 0;

  for (int i = 0; i < dim; i++)
  {
    leg->Average =  leg->Average + leg->TarrayPoint[i];
  }

  leg->FSR_Toe_Average = fsr(leg->fsr_sense_Toe);
  //  leg->Average_Volt = leg->FSR_Average;

  leg->FSR_Heel_Average = fsr(leg->fsr_sense_Heel);
  //  leg->Average_Volt_Heel = leg->FSR_Average_Heel;


  leg->FSR_Combined_Average = (leg->FSR_Toe_Average + leg->FSR_Heel_Average);

  leg->Average_Trq = leg->Average / dim;

  if (FLAG_TWO_TOE_SENSORS)
  {
    leg->p_steps->curr_voltage = leg->FSR_Combined_Average;
  }
  else {
    leg->p_steps->curr_voltage = leg->FSR_Toe_Average;
  }
  leg->p_steps->torque_average = leg->Average / dim;

}


//----------------------------------------------------------------------------------

void calculate_averages() {
  calculate_leg_average(left_leg);
  calculate_leg_average(right_leg);

  if (FLAG_PRINT_TORQUES) {
    Serial.print("LEFT [");
    for (int i = 0; i < dim; i++) {
      Serial.print(left_leg->TarrayPoint[i]);
      Serial.print(" , ");
    }
    Serial.print(" ] Average: ");
    Serial.println(left_leg->Average_Trq);
    Serial.print("RIGHT [");
    for (int i = 0; i < dim; i++) {
      Serial.print(right_leg->TarrayPoint[i]);
      Serial.print(" , ");
    }
    Serial.print(" ] Average: ");
    Serial.println(right_leg->Average_Trq);
  }

}

//----------------------------------------------------------------------------------

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

//----------------------------------------------------------------------------------

void check_Balance_Baseline() {
  if (FLAG_BALANCE_BASELINE) {
    Balance_Baseline();
  }
  if (FLAG_STEADY_BALANCE_BASELINE) {
    Steady_Balance_Baseline();
  }

}


//----------------------------------------------------------------------------------

void rotate_motor() {


  //  // modification to check the pid
  //  if (FLAG_PID_VALS) {
  //
  //    pid(left_leg, left_leg->Average_Trq);
  //    pid(right_leg, right_leg->Average_Trq);
  //
  //    Serial.print("LEFT PID INPUT:");
  //    Serial.print(left_leg->Input);
  //    Serial.print(" , AVG: ");
  //    Serial.print(left_leg->Average_Trq);
  //    Serial.print(" , VOL: ");
  //    Serial.println(left_leg->Vol);
  //    Serial.print("RIGHT PID INPUT:");
  //    Serial.print(right_leg->Input);
  //    Serial.print(" , AVG: ");
  //    Serial.print(right_leg->Average_Trq);
  //    Serial.print(" , VOL: ");
  //    Serial.println(right_leg->Vol);
  //
  //  }
  //  // end modification


  if (stream == 1)
  {
    if (streamTimerCount >= 5)
    {
      //            if ( analogRead(A11) * 3.3 / 4096 > 2.5) {
      //      if(analogRead(A11) * 3.3 / 4096 > 2.5 && flag_done_once_bt == false){
      counter_msgs++;
      send_data_message_wc();
      //            }
      streamTimerCount = 0;
    }

    if (streamTimerCount == 1 && flag_auto_KF == 1) {
      Auto_KF(left_leg);
      Auto_KF(right_leg);
    }


    streamTimerCount++;

    pid(left_leg, left_leg->Average_Trq);
    pid(right_leg, right_leg->Average_Trq);


    // modification to check the pid
    if (FLAG_PID_VALS) {

      Serial.print("LEFT PID INPUT:");
      Serial.print(left_leg->Input);
      Serial.print(" , AVG: ");
      Serial.print(left_leg->Average_Trq);
      Serial.print(" , VOL: ");
      double cane = (left_leg->Vol);
      Serial.println(cane - zero);
      Serial.print("RIGHT PID INPUT:");
      Serial.print(right_leg->Input);
      Serial.print(" , AVG: ");
      Serial.print(right_leg->Average_Trq);
      cane = (right_leg->Vol);
      Serial.print(" , VOL: ");
      Serial.println(cane - zero);

    }
    // end modification

    state_machine(left_leg);  //for LL
    state_machine(right_leg);  //for RL

    if (Control_Mode == 2) {}
    else {
      set_2_zero_if_steady_state();
    }

    left_leg->N3 = Control_Adjustment(left_leg, left_leg->state, left_leg->state_old, left_leg->p_steps,
                                      left_leg->N3, left_leg->New_PID_Setpoint, left_leg->p_Setpoint_Ankle,
                                      left_leg->p_Setpoint_Ankle_Pctrl, Control_Mode, left_leg->Prop_Gain,
                                      left_leg->FSR_baseline_FLAG, &left_leg->FSR_Ratio, &left_leg->Max_FSR_Ratio);
    right_leg->N3 = Control_Adjustment(right_leg, right_leg->state, right_leg->state_old, right_leg->p_steps,
                                       right_leg->N3, right_leg->New_PID_Setpoint, right_leg->p_Setpoint_Ankle,
                                       right_leg->p_Setpoint_Ankle_Pctrl, Control_Mode, right_leg->Prop_Gain,
                                       right_leg->FSR_baseline_FLAG, &right_leg->FSR_Ratio, &right_leg->Max_FSR_Ratio);
  }// end if stream==1
}


//----------------------------------------------------------------------------------


void reset_starting_parameters() {
  //Reset the starting values
  reset_leg_starting_parameters(left_leg);
  reset_leg_starting_parameters(right_leg);

  FLAG_TWO_TOE_SENSORS = OLD_FLAG_TWO_TOE_SENSORS;
}


//----------------------------------------------------------------------------------

void reset_leg_starting_parameters(Leg* leg) {
  leg->p_steps->count_plant = 0;
  leg->p_steps->n_steps = 0;
  leg->p_steps->flag_start_plant = false;
  leg->p_steps->flag_take_average = false;
  leg->p_steps->flag_N3_adjustment_time = false;
  leg->p_steps->flag_take_baseline = false;
  leg->p_steps->torque_adj = false;

  leg->N3 = N3;
  leg->N2 = N2;
  leg->N1 = N1;

  leg->p_steps->perc_l = 0.5;
  leg->activate_in_3_steps = 1;
  leg->Previous_Setpoint_Ankle = 0;

  leg->coef_in_3_steps = 0;
  leg->num_3_steps = 0;

  leg->first_step = 1;
  counter_msgs = 0;
  //  leg->BIO_BASELINE_FLAG=false;
  leg->Heel_Strike_Count = 0;
  leg->Heel_Strike = 0;
  leg->NO_Biofeedback = true;
}
