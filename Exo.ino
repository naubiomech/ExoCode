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
// 1 step = 0N
// 2 steps = 2N
// 3 steps = 4N
// 4 steps = 6N
// 5 steps = 8N
// 6 steps = 10N
//
// Several parameters can be modified thanks to the Receive and Transmit functions
#define VERSION 314
#define BOARD_VERSION DUAL_BOARD_REV6
//The digital pin connected to the motor on/off swich
const unsigned int zero = 2048; //1540;

#include <ArduinoBLE.h>
#include <elapsedMillis.h>
#include <PID_v2.h>
#include <Wire.h>

#include "Parameters.h"
#include "Board.h"
#include "Leg.h"
#include "Reference_ADJ.h"
#include "Msg_functions.h"
#include "Auto_KF.h"
#include "Variables.h"
#include "resetMotorIfError.h"
#include "ATP.h"
#include "Trial_Data.h"

bool DEBUG{false};
//----------------------------------------------------------------------------------


// Initialize the system
void setup()
{
  if (DEBUG) {Serial.println("Started");}

  //Nano's internal BLE module
  setupBLE();

  //Start Serial
  Serial.begin(9600);

  //set the resolution
  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  //initialize the leg objects
  initialize_left_leg(left_leg);
  initialize_right_leg(right_leg);

  //LED Stuff if Desired

  // set pin mode for motor pin
  pinMode(onoff, OUTPUT); //Enable disable the motors
  digitalWrite(onoff, LOW);

  // Initialize power monitor settings
  #define WireObj Wire
  //Setting both address pins to GND defines the slave address
  WireObj.begin(); //Initialize the I2C protocol on SDA1/SCL1 for Teensy 4.1, or SDA0/SCL0 on Teensy 3.6
  WireObj.beginTransmission(INA219_ADR); //Start talking to the INA219
  WireObj.write(INA219_CAL); //Write the target as the calibration register
  WireObj.write(Cal);        //Write the calibration value to the calibration register
  WireObj.endTransmission(); //End the transmission and calibration
  delay(100);

  int startVolt = readBatteryVoltage(); //Read the startup battery voltage
  Serial.println(startVolt);
  batteryData[0] = startVolt;
  send_command_message('~', batteryData, 1); //Communicate battery voltage to operating hardware, needs fixing!

  // Torque cal
  torque_calibration(); //Sets a torque zero on startup  

  if (DEBUG) {Serial.println("Setup complete");}
  startMillis = millis();
}

//----------------------------------------------------------------------------------

void callback()//executed every 1ms
{
  if (DEBUG) {Serial.println("Callback");}
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

  if (DEBUG) {Serial.println("Done with callback");}
}// end callback
//----------------------------------------------------------------------------------
// Function that is repeated in loop
void loop()
{
  if (DEBUG) {Serial.println("In loop");}
  //Looks for updates
  BLE.poll();
  
  currentMillis = millis();
  if ((currentMillis - startMillis) >= callBackPeriod)
  {
    callback();
    startMillis = currentMillis;
  }
  if (DEBUG) {Serial.println("Out loop");}
}// end void loop
//---------------------------------------------------------------------------------

void calculate_leg_average(Leg* leg) {
  //Calc the average value of Torque
  //Shift the arrays
  for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
    leg->TarrayPoint[j] = leg->TarrayPoint[j - 1];                //Puts the element in the following memory space into the current memory space
    leg->SpeedArrayPoint[j] = leg->SpeedArrayPoint[j - 1];
  }
  //Get the torque
  leg->TarrayPoint[0] = get_torq(leg);
  leg->FSR_Toe_Average = 0;
  leg->FSR_Heel_Average = 0;
  leg->Average = 0;

  //Motor Speed
  leg->SpeedArrayPoint[0] = ankle_speed(leg->motor_speed_pin);
  leg->AverageSpeed = 0;

  for (int i = 0; i < dim; i++)
  {
    leg->Average =  leg->Average + leg->TarrayPoint[i];
    leg->AverageSpeed = leg->AverageSpeed + leg->SpeedArray[i];
  }

  leg->Average_Trq = leg->Average / dim;
  leg->AverageSpeed = leg->AverageSpeed / dim;
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
  if (DEBUG) {
    Serial.println("In calculate_averages()");
  }
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
  if (DEBUG) {
    Serial.println("Out calculate_averages()");
  }
}

//----------------------------------------------------------------------------------

void check_FSR_calibration() {
  if (DEBUG) {Serial.println("In check_FSR_calibration");}
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
  if (DEBUG) {Serial.println("Out check_FSR_calibration");}
}

//----------------------------------------------------------------------------------
// check if some data about the balance baseline exists and transmit them to the gui
void check_Balance_Baseline() {
  if (DEBUG) {Serial.println("In check_Balance_Baseline()");}
  if (FLAG_BALANCE_BASELINE) {
    Balance_Baseline();
  }
  if (FLAG_STEADY_BALANCE_BASELINE) {
    Steady_Balance_Baseline();
  }
  if (DEBUG) {Serial.println("Out rotate_motor()");}
}


//----------------------------------------------------------------------------------

void rotate_motor() {
  if (DEBUG) {Serial.println("In rotate_motor()");}
  // send the data message, adapt KF if required, apply the PID, apply the state machine,
  //adjust some control parameters as a function of the control strategy decided (Control_Adjustment)

  if (stream == 1)
  {
    if (DEBUG) {Serial.println("In stream if");}
    pid(left_leg, left_leg->Average_Trq);
    pid(right_leg, right_leg->Average_Trq);


    if (streamTimerCount >= streamTimerCountNum) // every streamTimerCountNum*2ms
    {
      if (DEBUG) {Serial.println("In streamTimerCount if");}
      counter_msgs++;
      send_data_message_wc();
      streamTimerCount = 0;
      if (DEBUG) {Serial.println("Out stream if");}    
    }

    if (voltageTimerCount >= 15000 * 2) { //every 30 seconds
      if (DEBUG) {Serial.println("In voltageTimerCount if");}
      int batteryVoltage = readBatteryVoltage();
      Serial.println(batteryVoltage);
      batteryData[0] = batteryVoltage;
      send_command_message('~', batteryData, 1); //Communicate battery voltage to operating hardware
      voltageTimerCount = 0;

      if (DEBUG) {Serial.println("Out voltageTimerCount if");}
    }

    if (streamTimerCount == 1 && flag_auto_KF == 1) {
      if (DEBUG) {Serial.println("In flag_auto_KF if");}
      Auto_KF(left_leg, Control_Mode);
      Auto_KF(right_leg, Control_Mode);
      if (DEBUG) {Serial.println("Out flag_auto_KF if");}
    }


    streamTimerCount++;
    voltageTimerCount++;




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
    if (DEBUG) {Serial.println("Check States");}
    state_machine(left_leg);  //for LL
    state_machine(right_leg);  //for RL
    if (DEBUG) {Serial.println("Done Check states");}
    
    if (DEBUG) {Serial.println("Reference Millis");}
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

    if (DEBUG) {Serial.println("God's code");}
    // When I first wrote this only God and I knew what it did. Now only God knows. Need to go through this again. GO 9/17/20
    if ((Control_Mode == 3 || Control_Mode == 6) && (abs(left_leg->Dorsi_Setpoint_Ankle) > 0 || abs(left_leg->Previous_Dorsi_Setpoint_Ankle) > 0) && left_leg->state == 1) { //GO 4/22/19
      left_leg->PID_Setpoint = left_leg->New_PID_Setpoint;   //Brute force the dorsiflexion set point to proportional control
    } else if ((Control_Mode == 3 || Control_Mode == 6) && (abs(right_leg->Dorsi_Setpoint_Ankle) > 0 || abs(right_leg->Previous_Dorsi_Setpoint_Ankle) > 0) && right_leg->state == 1) {
      right_leg->PID_Setpoint = right_leg->New_PID_Setpoint; //Brute force the dorsiflexion set point to proportional control
    } else {};
    if (DEBUG) {Serial.println("End God's code");}

    int left_scaling_index = 0;
    int right_scaling_index = 0;

    if (Control_Mode == 5) {
      if (DEBUG) {Serial.println("In control_mode 5 if");}



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


    if (DEBUG) {Serial.println("End control mode 5 if");}
    }



    if (Control_Mode == 2) {}
    else {
      //set_2_zero_if_steady_state();
    }
    if (DEBUG) {Serial.println("Control Adjustments");}
    left_leg->N3 = Control_Adjustment(left_leg, left_leg->state, left_leg->state_old, left_leg->p_steps,
                                      left_leg->N3, left_leg->New_PID_Setpoint, left_leg->p_Setpoint_Ankle,
                                      left_leg->p_Setpoint_Ankle_Pctrl, Control_Mode, left_leg->Prop_Gain,
                                      left_leg->FSR_baseline_FLAG, &left_leg->FSR_Ratio, &left_leg->Max_FSR_Ratio);
    right_leg->N3 = Control_Adjustment(right_leg, right_leg->state, right_leg->state_old, right_leg->p_steps,
                                       right_leg->N3, right_leg->New_PID_Setpoint, right_leg->p_Setpoint_Ankle,
                                       right_leg->p_Setpoint_Ankle_Pctrl, Control_Mode, right_leg->Prop_Gain,
                                       right_leg->FSR_baseline_FLAG, &right_leg->FSR_Ratio, &right_leg->Max_FSR_Ratio);
    if (DEBUG) {Serial.println("Out stream if");}
  }// end if stream==1
  if (DEBUG) {Serial.println("Out rotate_motor()");}  
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
