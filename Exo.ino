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
#define BOARD_VERSION QUAD_BOARD
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
#include "Auto_KF.h"
#include "Auto_KF_Knee.h"  //  SS  9/18/2019
#include <Metro.h>
#include "Variables.h"
#include "Board.h"
#include "resetMotorIfError.h"
#include "ATP.h"
#include <i2c_t3.h> //  SS  8/17/2020
#include "AdafruitBNO055.h" //  SS  8/17/2020
#include <Filters.h> //  SS  10/25/2020
//----------------------------------------------------------------------------------


 //  SS  8/17/2020
#define IS_I2C0 true
#define IS_I2C1 false
Adafruit_BNO055 Left_ThighIMU = Adafruit_BNO055(IS_I2C0, IS_I2C1);// IMU slot 0 //  SS  8/17/2020

#define IS_I2C0 false
#define IS_I2C1 true
Adafruit_BNO055 Left_ShankIMU = Adafruit_BNO055(IS_I2C0, IS_I2C1, -1, BNO055_ADDRESS_A);// IMU slot 1 //  SS  8/17/2020

#define IS_I2C0 false
#define IS_I2C1 false
Adafruit_BNO055 Left_FootIMU = Adafruit_BNO055(IS_I2C0, IS_I2C1, -1, BNO055_ADDRESS_A);// IMU slot 2 //  SS  8/17/2020


FilterTwoPole Filter( LOWPASS_BUTTERWORTH, 3 );


 


// Initialize the system
void setup()
{
  // set the interrupt timer
  Timer1.initialize(2000);         // initialize timer1, and set a 2 ms period *note this is 2k microseconds*
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  // enable bluetooth
  bluetooth.begin(115200);
  Serial.begin(115200);

  
  //set the resolution
  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  //initialize the leg objects
  initialize_left_leg(left_leg);
  initialize_right_leg(right_leg);

  // set the led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // set pin mode for motor pin
  pinMode(onoff, OUTPUT); //Enable disable the motors
  digitalWrite(onoff, LOW);

  // Fast torque calibration
  torque_calibration();

  digitalWrite(LED_PIN, HIGH);
  
  // detect IMU //  SS  8/17/2020
  DetectKneeIMU = detect_IMU(Left_ThighIMU);
  DetectAnkleIMU = detect_IMU(Left_ShankIMU);
  DetectAnkleIMU = detect_IMU(Left_FootIMU);

  p_FirCoeff = FirCoeff;


  Serial.begin(115200);
  delay(3000);
  Serial.println("Start...");
  Serial.flush();
  if (!Left_FootIMU.begin())
    Serial.println("Error");
  Serial.println("OK");
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

 
  if (slowThisDown.check() == 1) // If the time passed is over 1ms is a true statement
  {
    if (bluetooth.available() > 0) // If bluetooth buffer contains something
    {
      receive_and_transmit();       //Recieve and transmit
    }

    slowThisDown.reset();     //Resets the interval counter
  }

  //  // apply biofeedback, if it is not activated return void
  biofeedback();

  // read angle of segment from IMU//  SS  8/17/2020

//  for (int i=0; i <= 7; i++){
//    left_leg->PrevAngle_Thigh[i] = left_leg->PrevAngle_Thigh[i+1];
//    left_leg->PrevAngle_Shank[i] = left_leg->PrevAngle_Shank[i+1];
//    left_leg->PrevAngle_Foot[i] = left_leg->PrevAngle_Foot[i+1];
//  }
  left_leg->PrevAngle_Thigh[8] = left_leg->Angle_Thigh;// Angle in rad
  left_leg->Angle_Thigh = (3.14159/2) - readAngle(Left_ThighIMU);// Angle in rad
  Filter.input(left_leg->Angle_Thigh);
//  left_leg->AngleAve_Thigh = left_leg->Angle_Thigh;

  left_leg->PrevAngle_Shank[8] = left_leg->Angle_Shank;// Angle in rad
  left_leg->Angle_Shank = (3.14159/2) - readAngle(Left_ShankIMU);// Angle in rad
  Filter.input(left_leg->Angle_Shank);
//  left_leg->AngleAve_Shank = left_leg->Angle_Shank;

  left_leg->PrevAngle_Foot[8] = left_leg->Angle_Foot;// Angle in rad
  left_leg->Angle_Foot = readAngle(Left_FootIMU);// Angle in rad
  Filter.input(left_leg->Angle_Foot);
//  left_leg->AngleAve_Foot = left_leg->Angle_Foot;
  
//  for (int i=0; i <= 8; i++){
//    left_leg->AngleAve_Thigh += left_leg->PrevAngle_Thigh[i];
//    left_leg->AngleAve_Shank += left_leg->PrevAngle_Shank[i];
//    left_leg->AngleAve_Foot += left_leg->PrevAngle_Foot[i];
////  }
//  left_leg->AngleAve_Thigh = left_leg->AngleAve_Thigh/10;
//  Filter.input( left_leg->AngleAve_Thigh );
//
//  left_leg->AngleAve_Shank = left_leg->AngleAve_Shank/10;
//  Filter.input( left_leg->AngleAve_Shank );
//
//  left_leg->AngleAve_Foot = left_leg->AngleAve_Foot/10;
//  Filter.input( left_leg->AngleAve_Foot );
//
  dt = (millis() - t) / 1000;

  for (int i=0; i <= 12; i++){
    left_leg->PrevAngularVel_Thigh[i] = left_leg->PrevAngularVel_Thigh[i+1];
    left_leg->PrevAngularVel_Shank[i] = left_leg->PrevAngularVel_Shank[i+1];
    left_leg->PrevAngularVel_Foot[i] = left_leg->PrevAngularVel_Foot[i+1];
  }
  left_leg->PrevAngularVel_Thigh[13] = left_leg->AngularVel_Thigh;// rad/s
//  left_leg->AngularVel_Thigh = readAngularVel(Left_ThighIMU);// rad/s
  left_leg->AngularVel_Thigh = (left_leg->Angle_Thigh - left_leg->PrevAngle_Thigh[8])/(dt*2);
  Filter.input( left_leg->AngularVel_Thigh );
  if ((abs(left_leg->AngularVel_Thigh - left_leg->PrevAngularVel_Thigh[13]) > abs(20*left_leg->PrevAngularVel_Thigh[13])) && (left_leg->PrevAngularVel_Thigh[13] !=0)){// SS  11/8/2020
    left_leg->AngularVel_Thigh =  left_leg->PrevAngularVel_Thigh[13];// SS  11/8/2020
  }
  left_leg->PrevAngularVelAve_Thigh = left_leg->AngularVelAve_Thigh;
  left_leg->AngularVelAve_Thigh = left_leg->AngularVel_Thigh;

  left_leg->PrevAngularVel_Shank[13] = left_leg->AngularVel_Shank;// rad/s
//  left_leg->AngularVel_Shank = readAngularVel(Left_ShankIMU);// rad/s
  left_leg->AngularVel_Shank = (left_leg->Angle_Shank - left_leg->PrevAngle_Shank[8])/(dt*2);
  Filter.input( left_leg->AngularVel_Shank );
  if ((abs(left_leg->AngularVel_Shank - left_leg->PrevAngularVel_Shank[13]) > abs(20*left_leg->PrevAngularVel_Shank[13])) && (left_leg->PrevAngularVel_Shank[13] !=0)){// SS  11/8/2020
    left_leg->AngularVel_Shank =  left_leg->PrevAngularVel_Shank[13];// SS  11/8/2020
  }
  left_leg->PrevAngularVelAve_Shank = left_leg->AngularVelAve_Shank;
  left_leg->AngularVelAve_Shank = left_leg->AngularVel_Shank;

  left_leg->PrevAngularVel_Foot[13] = left_leg->AngularVel_Foot;// rad/s
//  left_leg->AngularVel_Foot = readAngularVel(Left_FootIMU);// rad/s
  left_leg->AngularVel_Foot = (left_leg->Angle_Foot - left_leg->PrevAngle_Foot[8])/(dt*2);
  Filter.input( left_leg->AngularVel_Foot );
  if ((abs(left_leg->AngularVel_Foot - left_leg->PrevAngularVel_Foot[13]) > abs(20*left_leg->PrevAngularVel_Foot[13])) && (left_leg->PrevAngularVel_Foot[13] !=0)){// SS  11/8/2020
    left_leg->AngularVel_Foot =  left_leg->PrevAngularVel_Foot[13];// SS  11/8/2020
  }
  left_leg->PrevAngularVelAve_Foot = left_leg->AngularVelAve_Foot;
  left_leg->AngularVelAve_Foot = left_leg->AngularVel_Foot;
  
  for (int i=0; i <= 13; i++){
    left_leg->AngularVelAve_Thigh += left_leg->PrevAngularVel_Thigh[i];
    left_leg->AngularVelAve_Shank += left_leg->PrevAngularVel_Shank[i];
    left_leg->AngularVelAve_Foot += left_leg->PrevAngularVel_Foot[i];
  }
  left_leg->AngularVelAve_Thigh = left_leg->AngularVelAve_Thigh/15;
  Filter.input( left_leg->AngularVelAve_Thigh );

  left_leg->AngularVelAve_Shank = left_leg->AngularVelAve_Shank/15;
  Filter.input( left_leg->AngularVelAve_Shank );

  left_leg->AngularVelAve_Foot = left_leg->AngularVelAve_Foot/15;
  Filter.input( left_leg->AngularVelAve_Foot );

  
  


  for (int i=0; i <= 12; i++){
    left_leg->PrevAngularAcc_Thigh[i] = left_leg->PrevAngularAcc_Thigh[i+1];
    left_leg->PrevAngularAcc_Shank[i] = left_leg->PrevAngularAcc_Shank[i+1];
    left_leg->PrevAngularAcc_Foot[i] = left_leg->PrevAngularAcc_Foot[i+1];
  }
  left_leg->PrevAngularAcc_Thigh[13] = left_leg->AngularAcc_Thigh;// rad/s
  left_leg->AngularAcc_Thigh = (left_leg->AngularVelAve_Thigh - left_leg->PrevAngularVelAve_Thigh) / dt;// rad/s^2
  Filter.input( left_leg->AngularAcc_Thigh );
  left_leg->AngularAccAve_Thigh = left_leg->AngularAcc_Thigh;

  left_leg->PrevAngularAcc_Shank[13] = left_leg->AngularAcc_Shank;// rad/s
  left_leg->AngularAcc_Shank = (left_leg->AngularVelAve_Shank - left_leg->PrevAngularVelAve_Shank) / dt;// rad/s^2
  Filter.input( left_leg->AngularAcc_Shank);
  left_leg->AngularAccAve_Shank = left_leg->AngularAcc_Shank;

  left_leg->PrevAngularAcc_Foot[13] = left_leg->AngularAcc_Foot;// rad/s
  left_leg->AngularAcc_Foot = (left_leg->AngularVelAve_Foot - left_leg->PrevAngularVelAve_Foot) / dt;// rad/s^2
  Filter.input( left_leg->AngularAcc_Foot );
  left_leg->AngularAccAve_Foot = left_leg->AngularAcc_Foot;

  
  for (int i=0; i <= 13; i++){
    left_leg->AngularAccAve_Thigh += left_leg->PrevAngularAcc_Thigh[i];
    left_leg->AngularAccAve_Shank += left_leg->PrevAngularAcc_Shank[i];
    left_leg->AngularAccAve_Foot += left_leg->PrevAngularAcc_Foot[i];
  }
  left_leg->AngularAccAve_Thigh = left_leg->AngularAccAve_Thigh/15;
  Filter.input( left_leg->AngularAccAve_Thigh );

  left_leg->AngularAccAve_Shank = left_leg->AngularAccAve_Shank/15;
  Filter.input( left_leg->AngularAccAve_Shank );

  left_leg->AngularAccAve_Foot = left_leg->AngularAccAve_Foot/15;
  Filter.input( left_leg->AngularAccAve_Foot );
  
  
  right_leg->Angle_Thigh = 0;
  right_leg->Angle_Shank = 0;
  right_leg->Angle_Foot = 0;

  right_leg->PrevAngularVel_Thigh[24] = right_leg->AngularVel_Thigh;// rad/s
  right_leg->PrevAngularVel_Shank[24] = right_leg->AngularVel_Shank;// rad/s
  right_leg->PrevAngularVel_Foot[24] = right_leg->AngularVel_Foot;// rad/s

  right_leg->AngularVel_Thigh = 0;// rad/s
  right_leg->AngularVel_Shank = 0;// rad/s
  right_leg->AngularVel_Foot = 0;// rad/s

  right_leg->AngularVelAve_Thigh = 0;
  right_leg->AngularVelAve_Shank = 0;
  right_leg->AngularVelAve_Foot = 0;

  if (dt != 0){
  right_leg->AngularAcc_Thigh = (right_leg->AngularVel_Thigh - right_leg->PrevAngularVel_Thigh[24]) / dt;// rad/s^2
  right_leg->AngularAcc_Shank = (right_leg->AngularVel_Shank - right_leg->PrevAngularVel_Shank[24]) / dt;// rad/s^2
  right_leg->AngularAcc_Foot = (right_leg->AngularVel_Foot - right_leg->PrevAngularVel_Foot[24])  /dt;// rad/s^2
  }else{
    right_leg->AngularAcc_Thigh = 0;// rad/s^2
  right_leg->AngularAcc_Shank = 0;// rad/s^2
  right_leg->AngularAcc_Foot = 0;// rad/s^2
  }
  right_leg->AngularAccAve_Thigh = 0;
  right_leg->AngularAccAve_Shank = 0;
  right_leg->AngularAccAve_Foot = 0;
  
  t = millis();


  AnkleMomentEstimation(right_leg); // SS  10/26/2020
  AnkleMomentEstimation(left_leg); // SS  10/26/2020

  KneeMomentEstimation(right_leg); // SS  10/26/2020
  KneeMomentEstimation(left_leg); // SS  10/26/2020

  HipMomentEstimation(right_leg); // SS  10/26/2020
  HipMomentEstimation(left_leg); // SS  10/26/2020

  
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
    leg->TarrayPoint_Knee[j] = leg->TarrayPoint_Knee[j - 1];                //  SS  9/18/2019
  }

  //Get the torque
  leg->TarrayPoint[0] = get_torq(leg); 
  leg->TarrayPoint_Knee[0] = -get_torq_Knee(leg); //  SS  9/18/2019

  leg->FSR_Toe_Average = 0;
  leg->FSR_Heel_Average = 0;
  leg->Average = 0;
  leg->Average_K = 0;  //  SS  9/18/2019


  for (int i = 0; i < dim; i++)
  {
    leg->Average =  leg->Average + leg->TarrayPoint[i];
    leg->Average_K =  leg->Average_K + leg->TarrayPoint_Knee[i];   //  SS  9/18/2019
  }

  leg->Average_Trq = leg->Average / dim;
  leg->Average_Trq_Knee = leg->Average_K / dim;     //  SS  9/18/2019
  if (abs(leg->Average_Trq) > abs(leg->Max_Measured_Torque) && (leg->state == 3 || leg->state == 2)) {
    leg->Max_Measured_Torque = leg->Average_Trq;  //Get max measured torque during stance
  }
    //  SS  9/18/2019
  if (abs(leg->Average_Trq_Knee) > abs(leg->Max_Measured_Torque_Knee) &&  (leg->state == 3 || leg->state == 2)) {
    leg->Max_Measured_Torque_Knee = leg->Average_Trq_Knee;  //Get max measured torque during
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

  //  SS  9/18/2019
  if (abs(leg->TarrayPoint_Knee[dim]) > 25 && abs(leg->Average_Trq_Knee - leg->TarrayPoint_Knee[dim]) < 0.1) //When torque sensor is unplugged we see the same values for several seconds
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
  leg->p_steps->torque_average_K = leg->Average_K / dim;    //  SS  9/18/2019

  leg->FSR_Toe_Average = fsr(leg->fsr_sense_Toe);
  leg->FSR_Heel_Average = fsr(leg->fsr_sense_Heel);

  leg->FSR_Toe_Abs = leg->FSR_Toe_Average - leg->fsr_Toe_offset;  //  SS  11/8/2020
    if (leg->FSR_Toe_Abs < 0)
  leg->FSR_Toe_Abs = 0;
  leg->FSR_Heel_Abs = leg->FSR_Heel_Average - leg->fsr_Heel_offset;  //  SS  11/8/2020
  if (leg->FSR_Heel_Abs < 0)
    leg->FSR_Heel_Abs = 0;

  // in case of two toe sensors we use the combined averate, i.e. the sum of the averages.
  leg->FSR_Combined_Average = (leg->FSR_Toe_Average + leg->FSR_Heel_Average);

  leg->p_steps->curr_voltage_Toe = leg->FSR_Toe_Average;//  SS  9/18/2019
  leg->p_steps->curr_voltage_Heel = leg->FSR_Heel_Average;//  SS  9/18/2019

  leg->p_steps->curr_voltage = leg->FSR_Combined_Average;

}


//----------------------------------------------------------------------------------

void calculate_averages() {

  calculate_leg_average(left_leg);
  calculate_leg_average(right_leg);


//    Serial.print(" Angle:      ");
//    Serial.println(left_leg->Angle_Foot);
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
    if (streamTimerCount >= 5) // every 5*2ms, i.e. every .01s
    {
      counter_msgs++;
      send_data_message_wc();
      streamTimerCount = 0;
    }

    if (streamTimerCount == 1 && flag_auto_KF == 1) {
      Auto_KF(left_leg, Control_Mode);
      Auto_KF(right_leg, Control_Mode);
      Auto_KF_Knee(left_leg, Control_Mode);  //  SS  9/18/2019
      Auto_KF_Knee(right_leg, Control_Mode);  //  SS  9/18/2019
    }

    streamTimerCount++;

    pid(left_leg, left_leg->Average_Trq);
    pid(right_leg, right_leg->Average_Trq);

    pid_Knee(left_leg, -left_leg->Average_Trq_Knee);  //  SS  9/18/2019
    pid_Knee(right_leg, -right_leg->Average_Trq_Knee); //  SS  9/18/2019


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

    //    Serial.println("right_leg->p_steps->plant_peak_mean");
    //    Serial.println(right_leg->p_steps->plant_peak_mean);
    //    Serial.println("right_leg->p_steps->plant_peak_mean_Toe");
    //    Serial.println(right_leg->p_steps->plant_peak_mean_Toe);
    //    Serial.println("right_leg->p_steps->plant_peak_mean_Heel");
    //    Serial.println(right_leg->p_steps->plant_peak_mean_Heel);
    //    Serial.println("right_leg->Setpoint_Ankle_Pctrl");
    //    Serial.println(right_leg->Setpoint_Ankle_Pctrl);
    //    Serial.println("right_leg->PID_Setpoint");
    //    Serial.println(right_leg->PID_Setpoint);
    //    Serial.println("right_leg->PID_Setpoint_Knee");
    //    Serial.println(right_leg->PID_Setpoint_Knee);
    //    Serial.println("right_leg->Setpoint_Knee_Pctrl");
    //    Serial.println(right_leg->Setpoint_Knee_Pctrl);


    //
    //
    //    Serial.println("FLAG_TOE_HEEL_SENSORS");
    //    Serial.println(FLAG_TOE_HEEL_SENSORS);
    //
    //    Serial.println("Flag_Prop_Ctrl");
    //    Serial.println(Flag_Prop_Ctrl);
    //    Serial.println("flag_id");
    //    Serial.println(flag_id);
    //    Serial.println("flag_pivot");
    //    Serial.println(flag_pivot);
    //    Serial.println("Control Mode");
    //    Serial.println(Control_Mode);
    //
    //

    //    Serial.println("right_leg->coef_in_3_steps_Ankle");
    //    Serial.println(right_leg->coef_in_3_steps_Ankle);
    //    Serial.println("right_leg->num_3_steps_Ankle");
    //    Serial.println(right_leg->num_3_steps_Ankle);

    if ((left_leg->state == 3) && (left_leg->old_state == 1 || left_leg->old_state == 2)) {// SS 1/27/2020
      left_leg->state_3_start_time = millis();
    }

    if ((left_leg->state == 1 || left_leg->old_state == 2) && (left_leg->old_state == 3)) {// SS 1/27/2020
      left_leg->state_3_stop_time = millis();
    }

    if (left_leg->state_3_stop_time > left_leg->state_3_start_time) {
      left_leg->state_3_duration = left_leg->state_3_stop_time - left_leg->state_3_start_time;
    }
    if ((left_leg->state == 2) && (left_leg->old_state == 1 || left_leg->old_state == 3)) {// SS 1/27/2020
      left_leg->state_2_start_time = millis();
    }

    if ((left_leg->state == 1 || left_leg->old_state == 3) && (left_leg->old_state == 2)) {// SS 1/27/2020
      left_leg->state_2_stop_time = millis();
    }

    if (left_leg->state_2_stop_time > left_leg->state_2_start_time) {
      left_leg->state_2_duration = left_leg->state_2_stop_time - left_leg->state_2_start_time;
    }
    
    left_leg->old_state = left_leg->state;

    if ((right_leg->state == 3) && (right_leg->old_state == 1 || right_leg->old_state == 1)) {
      right_leg->state_3_start_time = millis();
    }
    if ((right_leg->state == 1 || right_leg->old_state == 2) && (right_leg->old_state == 3)) {// SS 1/27/2020
      right_leg->state_3_stop_time = millis();
    }

    if (right_leg->state_3_stop_time > right_leg->state_3_start_time) {
      right_leg->state_3_duration = right_leg->state_3_stop_time - right_leg->state_3_start_time;
    }
    if ((right_leg->state == 2) && (right_leg->old_state == 1 || right_leg->old_state == 3)) {// SS 1/27/2020
      right_leg->state_2_start_time = millis();
    }

    if ((right_leg->state == 1 || left_leg->old_state == 3) && (right_leg->old_state == 2)) {// SS 1/27/2020
      right_leg->state_2_stop_time = millis();
    }

    if (right_leg->state_2_stop_time > right_leg->state_2_start_time) {
      right_leg->state_2_duration = right_leg->state_2_stop_time - right_leg->state_2_start_time;
    }

    right_leg->old_state = right_leg->state;


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

      if ((right_leg->state == 3) && (right_leg->state_3_duration > 0)) {
        right_scaling_index = (millis() - right_leg->state_3_start_time) / (right_leg->state_3_duration / 100);
        if (right_scaling_index < 101) {
          right_leg->PID_Setpoint = ATP[101 + right_scaling_index];
        }
        else {
          right_leg->PID_Setpoint = 0;
        }
      }
    }





    if (Control_Mode == 2) {}
    else {
      set_2_zero_if_steady_state();
    }

      //  SS  9/18/2019
    left_leg->N3 = Control_Adjustment(left_leg, left_leg->state, left_leg->state_old, left_leg->p_steps,
                                      left_leg->N3, left_leg->New_PID_Setpoint, left_leg->p_Setpoint_Ankle_Pctrl,
                                      left_leg->New_PID_Setpoint_Knee, left_leg->p_Setpoint_Knee_Pctrl, Control_Mode, left_leg->Prop_Gain,
                                      left_leg->Angle_Thigh, left_leg->Angle_Shank, left_leg->Angle_Foot,
                                      left_leg->FSR_baseline_FLAG, &left_leg->FSR_Ratio, &left_leg->FSR_Ratio_Toe, &left_leg->INTEG_Ratio_Toe, &left_leg->FSR_Ratio_Heel, &left_leg->INTEG_Ratio_Heel,
                                      &left_leg->FSR_Ratio_HeelMinusToe, &left_leg->INTEG_Ratio_HeelMinusToe, &left_leg->FSR_Ratio_Ankle, &left_leg->INTEG_Ratio_Ankle, &left_leg->Moment_Ratio_Ankle, &left_leg->FSR_Ratio_Knee, &left_leg->INTEG_Ratio_Knee,
                                      &left_leg->Max_FSR_Ratio, &left_leg->Max_FSR_Ratio_Toe, &left_leg->Max_INTEG_Ratio_Toe, &left_leg->Max_FSR_Ratio_Heel, &left_leg->Max_INTEG_Ratio_Heel,
                                      &left_leg->Max_FSR_Ratio_HeelMinusToe, &left_leg->Max_INTEG_Ratio_HeelMinusToe, &left_leg->Max_FSR_Ratio_Ankle, &left_leg->Max_INTEG_Ratio_Ankle, &left_leg->Max_FSR_Ratio_Knee, &left_leg->Max_INTEG_Ratio_Knee);
    right_leg->N3 = Control_Adjustment(right_leg, right_leg->state, right_leg->state_old, right_leg->p_steps,
                                       right_leg->N3, right_leg->New_PID_Setpoint, right_leg->p_Setpoint_Ankle_Pctrl,
                                       right_leg->New_PID_Setpoint_Knee, right_leg->p_Setpoint_Knee_Pctrl, Control_Mode, right_leg->Prop_Gain,                                       
                                       right_leg->Angle_Thigh, right_leg->Angle_Shank, right_leg->Angle_Foot,
                                       right_leg->FSR_baseline_FLAG, &right_leg->FSR_Ratio, &right_leg->FSR_Ratio_Toe, &right_leg->INTEG_Ratio_Toe, &right_leg->FSR_Ratio_Heel, &right_leg->INTEG_Ratio_Heel,
                                       &right_leg->FSR_Ratio_HeelMinusToe, &right_leg->INTEG_Ratio_HeelMinusToe, &right_leg->FSR_Ratio_Ankle, &right_leg->INTEG_Ratio_Ankle, &right_leg->Moment_Ratio_Ankle, &right_leg->FSR_Ratio_Knee, &right_leg->INTEG_Ratio_Knee,
                                       &right_leg->Max_FSR_Ratio, &right_leg->Max_FSR_Ratio_Toe, &right_leg->Max_INTEG_Ratio_Toe, &right_leg->Max_FSR_Ratio_Heel, &right_leg->Max_INTEG_Ratio_Heel,
                                       &right_leg->Max_FSR_Ratio_HeelMinusToe, &right_leg->Max_INTEG_Ratio_HeelMinusToe, &right_leg->Max_FSR_Ratio_Ankle, &right_leg->Max_INTEG_Ratio_Ankle, &right_leg->Max_FSR_Ratio_Knee, &right_leg->Max_INTEG_Ratio_Knee);

  }// end if stream==1
}


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
  leg->activate_in_3_steps_Ankle = 1;
  leg->activate_in_3_steps_Knee = 1;  //  SS  9/18/2019
  leg->Previous_Setpoint_Ankle = 0;
  leg->Previous_Setpoint_Knee = 0;  //  SS  9/18/2019

  leg->coef_in_3_steps_Ankle = 0;
  leg->num_3_steps_Ankle = 0;

  leg->coef_in_3_steps_Knee = 0;
  leg->num_3_steps_Knee = 0;

  leg->first_step_Ankle = 1;
  leg->first_step_Knee = 1;
  counter_msgs = 0;
  leg->Heel_Strike_Count = 0;
  leg->score = 0;
  leg->Heel_Strike = 0;
  leg->NO_Biofeedback = true;
}
