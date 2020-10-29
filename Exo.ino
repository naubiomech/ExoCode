//A_Exo 3_1_3 This is the code for the Single board Ankle Exoskeleton -> A_EXO_s
//
// FSR sensors retrieve the sensor voltage related to the foot pressure.
// The state machine (Left and Right state machine) identify the participant status depending on the voltage.
// The torque reference is decided by the user (Matlab GUI) and sent as reference to the PID control.
// The PID control and data tranmission to the GUI are scheduled by an interrupt which interrupts the system every 2ms to do its job
//
// Sensor can be calibrated by the user and/or saved calibration can be loaded to speed up the setup.
//
// The torque reference can be smoothed by sigmoid functions or spline function
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
// Several parameters can be modified thanks to the Receive and Transmit functions
#define VERSION 314
#define BOARD_VERSION DUAL_BOARD_REV4
//The digital pin connected to the motor on/off swich
const unsigned int zero = 2048;//1540;

int j = 0;

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
#include "Auto_KF.h"
#include <Metro.h>
#include <Wire.h>
#include "Variables.h"
#include "Board.h"
#include "resetMotorIfError.h"
#include "ATP.h"
<<<<<<< HEAD
//#include "Wave.h"
#include "Step.h"
#include "Math.h"

=======
bool iOS_Flag = 0;
int streamTimerCountNum = 0;
>>>>>>> Models/Calibrations
//----------------------------------------------------------------------------------


// Initialize the system
void setup()
{
  // set the interrupt timer
  Serial.println("Started");
//  #if BOARD_VERSION == DUAL_BOARD_REV4  //Use timer interrupts for teensy 3.6
//    Timer1.initialize(2000);            // initialize timer1, and set a 2 ms period *note this is 2k microseconds*
//    Timer1.attachInterrupt(callback);   // attaches callback() as a timer overflow interrupt
//  #endif

  // enable bluetooth
  #if BOARD_VERSION == DUAL_BOARD_REV3
    #define bluetooth Serial8
  #elif BOARD_VERSION == DUAL_BOARD_REV4
    #define bluetooth Serial4
  #endif
  if (iOS_Flag == true) 
  {
    bluetooth.begin(9600);
    Serial.begin(9600);
    streamTimerCountNum = 25;
  }
  else if (!iOS_Flag) 
  {
    bluetooth.begin(115200);
    Serial.begin(115200);
    streamTimerCountNum = 5;
  }
  
  //set the resolution
  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  //initialize the leg objects
  initialize_left_leg(left_leg);
  initialize_right_leg(right_leg);

  // set the led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //Serial.println("LED SET");

  // set pin mode for motor pin
  pinMode(onoff, OUTPUT); //Enable disable the motors
  digitalWrite(onoff, LOW);
  //Serial.println("ONOFF SET");

  #if BOARD_VERSION == DUAL_BOARD_REV4
    pinMode(TRIGGER_PIN, OUTPUT); // Enable the trigger //SS  6/23/2020
    digitalWrite(TRIGGER_PIN, HIGH); //SS  6/23/2020
  #endif

  // Fast torque calibration
  //torque_calibration();
  //Serial.println("Torque Cal Done");

  digitalWrite(LED_PIN, HIGH);
  //Serial.println("Wrote LED_High");

  // Initialize power monitor settings
  #if BOARD_VERSION == DUAL_BOARD_REV3
    #define WireObj Wire1
  #elif BOARD_VERSION == DUAL_BOARD_REV4
    #define WireObj Wire
  #endif  
  pinMode(PWR_ADR_0, OUTPUT);
  pinMode(PWR_ADR_1, OUTPUT);
  digitalWrite(PWR_ADR_0, LOW); 
  digitalWrite(PWR_ADR_1, LOW); //Setting both address pins to GND defines the slave address
  WireObj.begin(); //Initialize the I2C protocol on SDA1/SCL1 for Teensy 4.1, or SDA0/SCL0 on Teensy 3.6
  WireObj.beginTransmission(INA219_ADR); //Start talking to the INA219
  WireObj.write(INA219_CAL); //Write the target as the calibration register
  WireObj.write(Cal);        //Write the calibration value to the calibration register
  WireObj.endTransmission(); //End the transmission and calibration
  delay(100);
  
  int startVolt = readBatteryVoltage(); //Read the startup battery voltage
  Serial.println(startVolt);
  //Send data message to iOS here 

<<<<<<< HEAD
  //calculateWave();
  calculateStep();
=======
  Serial.println("Setup complete");
>>>>>>> Models/Calibrations
  
}

//----------------------------------------------------------------------------------

void callback()//executed every 2ms
{
  // reset the motor drivers if you encounter an unexpected current peakz
  resetMotorIfError();

  // read FSR and torque values and calculate averages in case of necessity
  calculate_averages();

  // if the FSR calibration flag is true calibrate the FSR
  check_FSR_calibration();

  // apply the PID ctrl to the motors
  rotate_motor();

  // same of FSR but for the balance baseline
  check_Balance_Baseline();

  // if flag auto reconnect BT is 1, activate the autoreconnect anche check the led voltage
  if (FLAG_AUTO_RECONNECT_BT) {
  }

  // if flag biofeedback is 1 update the step length of the biofeedback
  if (FLAG_BIOFEEDBACK) {
    Freq = left_leg->Frequency;

    state_machine(left_leg);
    state_machine(right_leg);
    biofeedback_step_state(right_leg);
    biofeedback_step_state(left_leg);

  }//end if(Flag_biofeedback)
}// end callback
//----------------------------------------------------------------------------------
// Function that is repeated in loop
void loop()
{
  //#if BOARD_VERSION == DUAL_BOARD_REV3 //Timer based control loop doesn't work on teensy 4.1 (REV3)
  if (controlLoop.check() == 1)
  {
    callback();
    controlLoop.reset();
  }
  //#endif

  if (slowThisDown.check() == 1) // If the time passed is over 1ms is a true statement
  {
    
    if (bluetooth.available() > 0) // If bluetooth buffer contains something
    {
      Serial.println("Something to read");
      receive_and_transmit();       //Recieve and transmit
      
    }

    slowThisDown.reset();     //Resets the interval counter
  }

  //  // apply biofeedback, if it is not activated return void
  biofeedback();

  //if the stream is not activated reset the starting parameters
  if (stream != 1) // stream is 1 once you push start trial in the matlab gui, is 0 once you push end trial.
  {
    reset_starting_parameters();
    flag_done_once_bt = false;
  }
}// end void loop
//---------------------------------------------------------------------------------
//// Function of the biofeedback as a function of the error between the current knee angle during the heelstrike
////and a reference value (baseline), the Frequency of the sound is changed.
//
void biofeedback() {

  if (right_leg->NO_Biofeedback && left_leg->NO_Biofeedback) {
  } else {
    if (left_leg->BioFeedback_Baseline_flag) {

      state = digitalRead(LED_PIN);

      if (state == HIGH) {
        state = LOW;
      } else {
        state = HIGH;
      }
      digitalWrite(LED_PIN, state);
    }
    //
    //
    //    right_leg->start_time_Biofeedback = millis();
    //    tone(A17, 500, 100);

  }
  return;
}

//----------------------------------------------------------------------------------

void calculate_leg_average(Leg* leg) {
  //Calc the average value of Torque

  //Shift the arrays
  for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
    leg->TarrayPoint[j] = leg->TarrayPoint[j - 1];                //Puts the element in the following memory space into the current memory space
    leg->MotorSpeedArrayPoint[j] = leg->MotorSpeedArrayPoint[j-1];
    leg->AnkleAngleArrayPoint[j] = leg->AnkleAngleArrayPoint[j-1];
    leg->AnkleSpeedArrayPoint[j] = leg->AnkleSpeedArrayPoint[j-1];
  }
  //Get the torque
  leg->TarrayPoint[0] = get_torq(leg);
  leg->FSR_Toe_Average = 0;
  leg->FSR_Heel_Average = 0;
  leg->Average = 0;

  //Motor Speed
  leg->MotorSpeedArrayPoint[0] = motor_ankle_speed(leg->motor_speed_pin);
  leg->MotorAverageSpeed = 0;
  
  //Ankle Angle Sensor
  leg->AnkleAngleArrayPoint[0] = ankle_angle(leg);
  leg->AnkleAverageAngle = 0;
  
  for (int i = 0; i < dim; i++)
  {
    leg->Average =  leg->Average + leg->TarrayPoint[i];
    leg->MotorAverageSpeed = leg->MotorAverageSpeed + leg->MotorSpeedArray[i];
    leg->AnkleAverageAngle = leg->AnkleAverageAngle + leg->AnkleAngleArray[i]; 
  }

  leg->Average_Trq = leg->Average / dim;
  leg->MotorAverageSpeed = leg->MotorAverageSpeed / dim;
  leg->AnkleAverageAngle = leg->AnkleAverageAngle / dim;

  //leg->AnkleAverageSpeed = (leg->AnkleAverageAngle - leg->PrevAnkleAngle)/0.002; //Angular Velocity in deg/s
  //leg->PrevAnkleAngle = leg->AnkleAverageAngle;
  
  leg->AnkleSpeedArrayPoint[0] = (leg->AnkleAverageAngle - leg->PrevAnkleAngle)/0.002; //Angular velocity in degrees/s
  leg->AnkleAverageSpeed = 0;
  
  for (int i = 0; i< dim; i++) {
    //leg->AnkleAverageSpeed = leg->AnkleAverageSpeed + (i+1)*leg->AnkleSpeedArray[i]; //Weighted moving average
    leg->AnkleAverageSpeed = leg->AnkleAverageSpeed + leg->AnkleSpeedArray[i];
  }
  //leg->AnkleAverageSpeed = leg->AnkleAverageSpeed / (dim*(dim+1)/2); //Weighted moving average
  leg->AnkleAverageSpeed = leg->AnkleAverageSpeed / dim;  
  leg->PrevAnkleAngle = leg->AnkleAverageAngle;
  
  if (abs(leg->Average_Trq) > abs(leg->Max_Measured_Torque) && leg->state == 3) {
    leg->Max_Measured_Torque = leg->Average_Trq;  //Get max measured torque during stance
  }

  if (abs(leg->TarrayPoint[dim]) > 25 && abs(leg->Average_Trq - leg->TarrayPoint[dim]) < 0.1) //When torque sensor is unplugged we see the same values for several seconds
  {
    double old_L_state_L = leg->state;
    leg->state = 9;
    send_data_message_wc();

    digitalWrite(onoff, LOW);
    stream = 0;
    digitalWrite(LED_PIN, LOW);
    leg->state = old_L_state_L;
  }
  leg->p_steps->torque_average = leg->Average / dim;

  leg->FSR_Toe_Average = fsr(leg->fsr_sense_Toe);
  leg->FSR_Heel_Average = fsr(leg->fsr_sense_Heel);

  // in case of two toe sensors we use the combined averate, i.e. the sum of the averages.
  leg->FSR_Combined_Average = (leg->FSR_Toe_Average + leg->FSR_Heel_Average);

  leg->p_steps->curr_voltage_Toe = leg->FSR_Toe_Average;
  leg->p_steps->curr_voltage_Heel = leg->FSR_Heel_Average;
  
  if (FLAG_ONE_TOE_SENSOR)
  {
    leg->p_steps->curr_voltage = leg->FSR_Toe_Average;
  } else {
    leg->p_steps->curr_voltage = leg->FSR_Combined_Average;
  }

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

  // for the proportional control
  if (right_leg->FSR_baseline_FLAG) {
    take_baseline(right_leg, right_leg->state, right_leg->state_old, right_leg->p_steps, right_leg->p_FSR_baseline_FLAG);
  }
  if (left_leg->FSR_baseline_FLAG) {
    take_baseline(left_leg, left_leg->state, left_leg->state_old, left_leg->p_steps, left_leg->p_FSR_baseline_FLAG);
  }

}

//----------------------------------------------------------------------------------
// check if some data about the balance baseline exists and transmit them to the gui
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
  // send the data message, adapt KF if required, apply the PID, apply the state machine,
  //adjust some control parameters as a function of the control strategy decided (Control_Adjustment)

  if (stream == 1)
  {
    pid(left_leg, left_leg->Average_Trq);
    pid(right_leg, right_leg->Average_Trq);

    
    if (streamTimerCount >= streamTimerCountNum) // every streamTimerCountNum*2ms
    {
      counter_msgs++;
      send_data_message_wc();
      streamTimerCount = 0;
    }

    if (streamTimerCount >= 15000*2) { //every 30 seconds
      int batteryVoltage = readBatteryVoltage();
      //Send data message here
    }

    if (streamTimerCount == 1 && flag_auto_KF == 1) {
      Auto_KF(left_leg, Control_Mode);
      Auto_KF(right_leg, Control_Mode);
    }


    streamTimerCount++;

    


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

        if ((left_leg->state == 3) && ((left_leg->old_state == 1) || (left_leg->old_state == 2))) {   // TN 9/26/19
      left_leg->state_3_start_time = millis();
    }

    if (((left_leg->state == 1) || (left_leg->state == 2)) && (left_leg->old_state == 3)) {     // TN 9/26/19
      left_leg->state_3_stop_time = millis();
    }

    if (left_leg->state_3_stop_time > left_leg->state_3_start_time) { // SS 8/6/2020
      left_leg->state_3_duration = left_leg->state_3_stop_time - left_leg->state_3_start_time;
    }

    if ((left_leg->state == 1) && ((left_leg->old_state == 3) || (left_leg->old_state == 2))) {  // SS 8/6/2020
      left_leg->state_1_start_time = millis();
    }

    if (((left_leg->state == 3) || (left_leg->state == 2)) && (left_leg->old_state == 1)) { // SS 8/6/2020
      left_leg->state_1_stop_time = millis();
    }

    if (left_leg->state_1_stop_time > left_leg->state_1_start_time) { // SS 8/6/2020
      left_leg->state_1_duration = left_leg->state_1_stop_time - left_leg->state_1_start_time;
    }

    left_leg->old_state = left_leg->state;

    if ((right_leg->state == 3) && ((right_leg->old_state == 1) || (right_leg->old_state == 2))) {    // TN 9/26/19
      right_leg->state_3_start_time = millis();
    }
    else {
      if (((right_leg->state == 1) || (right_leg->state == 2)) && (right_leg->old_state == 3)) {   // TN 9/26/19

        right_leg->state_3_stop_time = millis();

        if (right_leg->state_3_stop_time > right_leg->state_3_start_time) { // SS 8/6/2020
          right_leg->state_3_duration = right_leg->state_3_stop_time - right_leg->state_3_start_time;
        }
      }
    }

    if ((right_leg->state == 1) && ((right_leg->old_state == 3) || (right_leg->old_state == 2))) {  // SS 8/6/2020
      right_leg->state_1_start_time = millis();
    }
    else {
      if (((right_leg->state == 3) || (right_leg->state == 2)) && (right_leg->old_state == 1)) { // SS 8/6/2020

        right_leg->state_1_stop_time = millis();

        if (right_leg->state_1_stop_time > right_leg->state_1_start_time) { // SS 8/6/2020
          right_leg->state_1_duration = right_leg->state_1_stop_time - right_leg->state_1_start_time;
        }
      }
    }

    right_leg->old_state = right_leg->state;

//    #if BOARD_VERSION == DUAL_BOARD_REV4
//      // Sending nerve stimulation tigger // SS 8/6/2020
//      if (STIM_ACTIVATED){
//        if (Trigger_left)  send_trigger(left_leg); //for left
//        else  send_trigger(right_leg);  //for right (the default is for right leg)
//        }
//    #endif    

    // When I first wrote this only God and I knew what it did. Now only God knows. Need to go through this again. GO 9/17/20
    if ((Control_Mode == 3 || Control_Mode == 6) && (abs(left_leg->Dorsi_Setpoint_Ankle) > 0 || abs(left_leg->Previous_Dorsi_Setpoint_Ankle) > 0) && left_leg->state == 1) { //GO 4/22/19
      left_leg->PID_Setpoint = left_leg->New_PID_Setpoint;   //Brute force the dorsiflexion set point to proportional control
    } else if ((Control_Mode == 3 || Control_Mode == 6) && (abs(right_leg->Dorsi_Setpoint_Ankle) > 0 || abs(right_leg->Previous_Dorsi_Setpoint_Ankle) > 0) && right_leg->state == 1) {
      right_leg->PID_Setpoint = right_leg->New_PID_Setpoint; //Brute force the dorsiflexion set point to proportional control
    } else {};

    int left_scaling_index = 0;
    int right_scaling_index = 0;

    if (Control_Mode == 5) {



      if ((left_leg->state == 3) && (left_leg->state_3_duration > 0)) {
        left_scaling_index = (millis() - left_leg->state_3_start_time) / (left_leg->state_3_duration / 100);
        if (left_scaling_index < 101) {
          left_leg->PID_Setpoint = ATP[left_scaling_index];
        }
        else {
          left_leg->PID_Setpoint = 0;
        }
      }
      else {
        left_leg->PID_Setpoint = 0;  // TN 9/25/19
      }

      if ((right_leg->state == 3) && (right_leg->state_3_duration > 0)) {
        right_scaling_index = (millis() - right_leg->state_3_start_time) / (right_leg->state_3_duration / 100);
        if (right_scaling_index < 101) {
          right_leg->PID_Setpoint = ATP[101 + right_scaling_index];
        }
        else {
          right_leg->PID_Setpoint = 0;
        }
      }
      else {
        right_leg->PID_Setpoint = 0;   // TN 8/20/19
      }



    }



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
//void send_trigger(Leg* leg) {   // Nerve stimulation trigger function // SS 8/6/2020
//
// 
//if (leg->state == 3){
//  leg->swing_counter = 0;
//  leg->stance_counter ++;
//  
//  if ((((millis() - leg->trig_time) > 1000) && ((leg->stance_counter == 1) || (leg->stance_counter > (((leg->state_3_duration * 2) / 3)/2)) )) || leg->Approve_trigger) {
//    leg->Approve_trigger = true;
//    if ((leg->stance_counter < 41) && (leg->trig_number == 1)) { //  Trigger at the start of stance phase
//      digitalWrite(TRIGGER_PIN, HIGH);
//      leg->Trigger = 1;
//      } else if ((leg->stance_counter > (((leg->state_3_duration * 2) / 3)/2)) && (leg->stance_counter < (40 + (((leg->state_3_duration * 2) / 3)/2))) && (leg->trig_number == 2)) { // Trigger at the 2/3 of stance phase
//        digitalWrite(TRIGGER_PIN, HIGH);
//        leg->Trigger = 2;
//        } else  {
//          digitalWrite(TRIGGER_PIN, LOW);
//          leg->Old_Trigger = leg->Trigger;
//          leg->Trigger = 0;
//          if (leg->Old_Trigger != 0){
//            leg->trig_number = 0;
//            leg->Approve_trigger = false;
//            }
//          }
//    }else{
//      digitalWrite(TRIGGER_PIN, LOW);
//      leg->Old_Trigger = leg->Trigger;
//      leg->Trigger = 0;
//      if (leg->Old_Trigger != 0){
//         leg->trig_number = 0;
//         leg->Approve_trigger = false;
//         }
//    }
//  } else if (leg->state == 1){
//        leg->stance_counter = 0;
//        leg->swing_counter ++;
//
//        if ((((millis() - leg->trig_time) > 1000) && ((leg->swing_counter < ((leg->state_1_duration / 3)/2))) || (leg->swing_counter > (((leg->state_1_duration / 3)/2)+20)) ) || leg->Approve_trigger) {
//          leg->Approve_trigger = true;
//          if ((leg->swing_counter > ((leg->state_1_duration / 3)/2))  &&  (leg->swing_counter < (((leg->state_1_duration / 3)/2)+40)) && (leg->trig_number ==  3)) { // Trigger at the 1/3 of swing phase
//            digitalWrite(TRIGGER_PIN, HIGH);
//            leg->Trigger = 3;
//            } else if ((leg->swing_counter > (((leg->state_1_duration * 2)/ 3)/2))  &&  (leg->swing_counter < ((((leg->state_1_duration * 2) / 3)/2)+40))  && (leg->trig_number == 4))  { // Trigger at the 2/3 of swing phase
//              digitalWrite(TRIGGER_PIN, HIGH); 
//              leg->Trigger = 4;
//              } else  {
//                digitalWrite(TRIGGER_PIN, LOW);
//                leg->Old_Trigger = leg->Trigger;
//                leg->Trigger = 0;
//                if (leg->Old_Trigger != 0){
//                  leg->trig_number = 0;
//                  leg->Approve_trigger = false;
//                  }
//                }
//          }else{
//            digitalWrite(TRIGGER_PIN, LOW);
//            leg->Old_Trigger = leg->Trigger;
//            leg->Trigger = 0;
//            if (leg->Old_Trigger != 0){
//              leg->trig_number = 0;
//              leg->Approve_trigger = false;
//              }
//            }
//        }
//    
//}

//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------


void reset_starting_parameters() {
  //Reset the starting values
  reset_leg_starting_parameters(left_leg);
  reset_leg_starting_parameters(right_leg);

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
  leg->Heel_Strike_Count = 0;
  leg->score = 0;
  leg->Heel_Strike = 0;
  leg->NO_Biofeedback = true;

   // SS 8/6/2020
  leg->trig1_counter = 0;
  leg->trig2_counter = 0;
  leg->trig3_counter = 0;
  leg->trig4_counter = 0;
  leg->stance_counter = 0; 
  leg->swing_counter = 0;
  leg->trig_time = 0;
  leg->trig_number = 0;
  leg->Approve_trigger = false;
  STIM_ACTIVATED = false;
  Trigger_left = false;
}
