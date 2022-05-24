//TMOTOR Nano
//Ensure that the proper motor is defined in akxMotor.h!!!

#define VERSION 314
#define BOARD_VERSION TMOTOR_REV1

#define CONTROL_LOOP_HZ           1000
#define SAMPLE_LOOP_MULT          2
#define CONTROL_TIME_STEP         1 / CONTROL_LOOP_HZ
#define COMMS_LOOP_HZ             25
//The digital pin connected to the motor on/off swich
const unsigned int zero = 2048;
bool motors_on = false;

float l_torque = 0;
float r_torque = 0;

#include <ArduinoBLE.h>
#include <elapsedMillis.h>
#include <PID_v2.h>
#include <Wire.h>
#include <SPI.h>
#include <mbed.h>
#include <rtos.h>
 
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
#include "Ambulation_SM.h"
#include "fault_detection.h"
#include "ema_filter.h"
#include "motor_utils.h"
#include "akxMotor.h"
//----------------------------------------------------------------------------------
rtos::Thread callback_thread(osPriorityNormal);
rtos::Thread read_thread(osPriorityNormal);

Ambulation_SM amb_sm;
uint32_t imuCounter = 0;

/* Holds the latest motor data */
motor_frame_t l_frame;
motor_frame_t r_frame;

void control_loop() {
  int count = 0;
  while (true) {
    //start = micros();
    //imuCounter++;
    //resetMotorIfError();
    calculate_averages();
    count++;

//    if (motors_on) {
////      detect_faults();
////      if (amb_sm.last_state == Walking) {
////          tracking_check(right_leg);
////          tracking_check(left_leg);
////          pid_check(right_leg);
////          pid_check(left_leg);
////      }
//      if (stream && (imuCounter > imuTimerCount)) {
//        //amb_sm.tick(stepper->steps);
//        imuCounter = 0;
//      }
//    }

    if (count == SAMPLE_LOOP_MULT)
    {
      count = 0;
      //rotate_motor();
    }
    rotate_motor();
    //time = micros()-start;
    //Serial.println(time);
    rtos::ThisThread::sleep_for(1000 / (CONTROL_LOOP_HZ));
  }
}


void read_loop(void)
{
  motor_frame_t temp_frame;
  while(true) {
    if(!akMotor.updateFrame(&r_frame, 5)) {
      //Serial.println("R False");
    }
    if(!akMotor.updateFrame(&l_frame, 1)) {
      //Serial.println("L False");
    }
    
    rtos::ThisThread::sleep_for(10);
  } //End While
} //End read_loop()


// Initialize the system
void setup()
{
  // set pin mode for motor pin
  //pinMode(onoff, OUTPUT); //Enable disable the motors
  //digitalWrite(onoff, LOW);

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
  digitalWrite(GREEN, HIGH);
  
  //Start Serial
  Serial.begin(500000);
  delay(100);

  //Nano's internal BLE module
  setupBLE();

  //set the resolution
  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  //initialize the leg objects
  initialize_left_leg(left_leg);
  initialize_right_leg(right_leg);


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
  batteryData[0] = startVolt/10;
  send_command_message('~', batteryData, 1); //Communicate battery voltage to operating hardware

  torque_calibration(); //Sets a torque zero on startup  

  //Initailize motor driver IC
  akMotor.init();
  akMotor.setMotorState(L_ID, false);
  akMotor.setMotorState(R_ID, false);
  
  //Initialize the standing/walking state detector and provide callback functions
  amb_sm.init();
  amb_sm.attach_fe_cb(Amb_SM_cbs::upon_standing);
  amb_sm.attach_re_cb(Amb_SM_cbs::upon_walking);
  
  right_leg->Prev_Trq = get_torq(right_leg); //Initial conditions for EMA torque signal filter
  left_leg->Prev_Trq = get_torq(left_leg); 

  //Starts the Control Loop thread
  callback_thread.start(control_loop);
  //read_thread.start(read_loop);
}


//----------------------------------------------------------------------------------
void loop()
{  
  callback_thread.set_priority(osPriorityNormal);
  //Looks for updates
  BLE.poll();
  callback_thread.set_priority(osPriorityAboveNormal);
  // if the FSR calibration flag is true calibrate the FSR
  check_FSR_calibration();
  // same of FSR but for the balance baseline
  check_Balance_Baseline();
  //Updates GUI
  update_GUI();
  //Puts the main thread to sleep for (1000 / Frequency) milliseconds
  rtos::ThisThread::sleep_for(1000 / COMMS_LOOP_HZ);
}// end void loop


//---------------------------------------------------------------------------------
void update_GUI() {
  //Real Time data
  if (stream)
    {
      counter_msgs++;
      callback_thread.set_priority(osPriorityNormal);
      send_data_message_wc();
      callback_thread.set_priority(osPriorityAboveNormal);
    }
    
  //Battery voltage and reset motor count data
  if (voltageTimerCount >= voltageTimerCountNum) {
    static int batteryVoltage = readBatteryVoltage();
    int filtered_voltage = ema_with_context(batteryVoltage, readBatteryVoltage(), 0.0001);
    batteryVoltage = filtered_voltage;
    batteryData[0] = filtered_voltage/10; //convert from milli
    callback_thread.set_priority(osPriorityNormal);
    send_command_message('~', batteryData, 1); //Communicate battery voltage to operating hardware
    voltageTimerCount = 0;
    //Motor reset Count
    //errorCount[0] = reset_count;
    //send_command_message('w', errorCount, 1);
    callback_thread.set_priority(osPriorityAboveNormal);
  }
  voltageTimerCount++;
}

void calculate_leg_average(Leg* leg, double alpha) {

  if (alpha <= 0.00000001) {
    //Calc the average value of Torque
    //Shift the arrays
    for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
    { // there are the number of spaces in the memory space minus 2 actions that need to be taken
      leg->TarrayPoint[j] = leg->TarrayPoint[j - 1];                //Puts the element in the following memory space into the current memory space
      leg->CurrentArrayPoint[j] = leg->CurrentArrayPoint[j - 1];
    }
    //Get the torque
    leg->TarrayPoint[0] = get_torq(leg);
    leg->Average = 0;

    //Motor current needs the same moving average as torque
    leg->CurrentArrayPoint[0] = current(leg->motor_current_pin);
    leg->AverageCurrent = 0;
  
    for (int i = 0; i < dim; i++)
    {
      leg->Average =  leg->Average + leg->TarrayPoint[i];
      leg->AverageCurrent = leg->AverageCurrent + leg->CurrentArray[i];
    }
    leg->Average_Trq = leg->Average / dim;
    leg->AverageCurrent = leg->AverageCurrent / dim;
    
  } else {
    // Torque Poll and EMA Filter
    leg->Average_Trq = ema_with_context(leg->Prev_Trq,get_torq(leg),alpha);
    leg->Prev_Trq = leg->Average_Trq;
  }
  
//  if (abs(leg->Average_Trq) > abs(leg->Max_Measured_Torque) && leg->state == 3) {
//      leg->Max_Measured_Torque = leg->Average_Trq;  //Get max measured torque during stance
//  }
  
  
  leg->p_steps->torque_average = leg->Average_Trq;
  
  // FSR Poll

  leg->FSR_Toe_Average = 0;
  leg->FSR_Heel_Average = 0;

  leg->FSR_Toe_Average = fsr(leg->fsr_sense_Toe);
//  leg->FSR_Heel_Average = fsr(leg->fsr_sense_Heel);

  // in case of two toe sensors we use the combined averate, i.e. the sum of the averages.
  leg->FSR_Combined_Average = (leg->FSR_Toe_Average + 0);

  leg->p_steps->curr_voltage_Toe = leg->FSR_Toe_Average;
//  leg->p_steps->curr_voltage_Heel = leg->FSR_Heel_Average;

  if (FLAG_ONE_TOE_SENSOR)
  {
    leg->p_steps->curr_voltage = leg->FSR_Toe_Average;
  } else {
    leg->p_steps->curr_voltage = leg->FSR_Combined_Average;
  }
}

//----------------------------------------------------------------------------------

void calculate_averages() {
  calculate_leg_average(left_leg, 0); //0.80);
  calculate_leg_average(right_leg, 0);//0.80);
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
    float r_vol = pid(right_leg, right_leg->Average_Trq);
    akMotor.map_and_apply(R_ID, r_vol, right_leg->sign);
    //akMotor.updateFrame(&akMotor.right_return,0.1);
    
    float l_vol = pid(left_leg, left_leg->Average_Trq);
    akMotor.map_and_apply(L_ID, l_vol, left_leg->sign);
    //akMotor.updateFrame(&akMotor.left_return,0.1);

    if (flag_auto_KF) {
      Auto_KF(left_leg, Control_Mode);
      Auto_KF(right_leg, Control_Mode);
    }

    state_machine(left_leg);  //for LL
    state_machine(right_leg);  //for RL

    double time = millis();
    if ((left_leg->state == 3) && ((left_leg->old_state == 1) || (left_leg->old_state == 2))) {   // TN 9/26/19
      left_leg->state_3_start_time = time;
    }

    if (((left_leg->state == 1) || (left_leg->state == 2)) && (left_leg->old_state == 3)) {     // TN 9/26/19
      left_leg->state_3_stop_time = time;
    }

    if (left_leg->state_3_stop_time > left_leg->state_3_start_time) { // SS 8/6/2020
      left_leg->state_3_duration = left_leg->state_3_stop_time - left_leg->state_3_start_time;
    }

    if ((left_leg->state == 1) && ((left_leg->old_state == 3) || (left_leg->old_state == 2))) {  // SS 8/6/2020
      left_leg->state_1_start_time = time;
    }

    if (((left_leg->state == 3) || (left_leg->state == 2)) && (left_leg->old_state == 1)) { // SS 8/6/2020
      left_leg->state_1_stop_time = time;
    }

    if (left_leg->state_1_stop_time > left_leg->state_1_start_time) { // SS 8/6/2020
      left_leg->state_1_duration = left_leg->state_1_stop_time - left_leg->state_1_start_time;
    }

    left_leg->old_state = left_leg->state;

    if ((right_leg->state == 3) && ((right_leg->old_state == 1) || (right_leg->old_state == 2))) {    // TN 9/26/19
      right_leg->state_3_start_time = time;
    }
    else {
      if (((right_leg->state == 1) || (right_leg->state == 2)) && (right_leg->old_state == 3)) {   // TN 9/26/19

        right_leg->state_3_stop_time = time;

        if (right_leg->state_3_stop_time > right_leg->state_3_start_time) { // SS 8/6/2020
          right_leg->state_3_duration = right_leg->state_3_stop_time - right_leg->state_3_start_time;
        }
      }
    }

    if ((right_leg->state == 1) && ((right_leg->old_state == 3) || (right_leg->old_state == 2))) {  // SS 8/6/2020
      right_leg->state_1_start_time = time;
    }
    else {
      if (((right_leg->state == 3) || (right_leg->state == 2)) && (right_leg->old_state == 1)) { // SS 8/6/2020

        right_leg->state_1_stop_time = time;

        if (right_leg->state_1_stop_time > right_leg->state_1_start_time) { // SS 8/6/2020
          right_leg->state_1_duration = right_leg->state_1_stop_time - right_leg->state_1_start_time;
        }
      }
    }
    right_leg->old_state = right_leg->state;

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
      //set_2_zero_if_steady_state();
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


void reset_cal_on_end_trial() {
  /*
  reset_starting_parameters();
  //Previous Torques
  left_leg->Previous_Setpoint_Ankle = 0;
  left_leg->Previous_Dorsi_Setpoint_Ankle = 0;
  right_leg->Previous_Setpoint_Ankle = 0;
  right_leg->Previous_Dorsi_Setpoint_Ankle = 0;
  //Current Torques
  left_leg->Setpoint_Ankle = 0;
  left_leg->Dorsi_Setpoint_Ankle = 0;
  right_leg->Setpoint_Ankle = 0;
  right_leg->Dorsi_Setpoint_Ankle = 0;
  //Torque Resets
  left_leg->coef_in_3_steps = 0;
  left_leg->activate_in_3_steps = 1;
  left_leg->first_step = 1;
  left_leg->num_3_steps = 0;
  left_leg->start_step = 0;
  right_leg->coef_in_3_steps = 0;
  right_leg->activate_in_3_steps = 1;
  right_leg->first_step = 1;
  right_leg->num_3_steps = 0;
  right_leg->start_step = 0;
  
  //Baseline
  left_leg->baseline_value = 0;
  right_leg->baseline_value = 0;
  
  //FSR
  left_leg->fsr_Toe_peak_ref = 0;
  right_leg->fsr_Toe_peak_ref = 0;
  
  left_leg->fsr_Toe_trough_ref = 1000;
  right_leg->fsr_Toe_trough_ref = 1000;

  //P-Steps
  left_leg->p_steps->plant_peak_mean = 0;
  right_leg->p_steps->plant_peak_mean = 0;
  
  left_leg->p_steps->peak = 0;
  right_leg->p_steps->peak = 0;
  
  left_leg->p_steps->count_plant_base = 0;
  right_leg->p_steps->count_plant_base = 0;

  right_leg->p_steps->flag_start_plant = false;
  left_leg->p_steps->flag_start_plant = false;

  //PID
  /*
  left_leg->PID_Setpoint = 0;
  left_leg->New_PID_Setpoint = 0;
  right_leg->PID_Setpoint = 0;
  right_leg->New_PID_Setpoint = 0;
  */
}
