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
#define BOARD_VERSION TWO_LEG_BOARD
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
#include "Auto_KF_Knee.h"  // TN 5/15/19
#include <Metro.h>
#include "Variables.h"
#include "Board.h"
#include "resetMotorIfError.h"
#include "ATP.h"
//----------------------------------------------------------------------------------


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
  }
  //Get the torque
  leg->AorK = 'A';  // TN 5/9/19
  leg->TarrayPoint[0] = get_torq(leg);  // TN 5/9/19
  leg->AorK = 'K';  // TN 5/9/19
  leg->TarrayPoint_Knee[0] = get_torq(leg); // TN 5/9/19

  leg->FSR_Toe_Average = 0;
  leg->FSR_Heel_Average = 0;
  leg->Average = 0;
  leg->Average_K = 0;  // TN 5/9/19


  for (int i = 0; i < dim; i++)
  {
    leg->Average =  leg->Average + leg->TarrayPoint[i];
    leg->Average_K =  leg->Average + leg->TarrayPoint_Knee[i];   // TN 5/9/19
  }

  leg->Average_Trq = leg->Average / dim;
  leg->Average_Trq_Knee = leg->Average_K / dim;     // TN 5/9/19
  if (abs(leg->Average_Trq) > abs(leg->Max_Measured_Torque) && leg->state == 3) {
    leg->Max_Measured_Torque = leg->Average_Trq;  //Get max measured torque during stance
  }
  // TN 5/9/19
  if (abs(leg->Average_Trq_Knee) > abs(leg->Max_Measured_Torque_Knee)) {
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
  leg->p_steps->torque_average = leg->Average / dim;
  leg->p_steps->torque_average_K = leg->Average_K / dim;    // TN 5/9/19

  leg->FSR_Toe_Average = fsr(leg->fsr_sense_Toe);
  leg->FSR_Heel_Average = fsr(leg->fsr_sense_Heel);

  // in case of two toe sensors we use the combined averate, i.e. the sum of the averages.
  leg->FSR_Combined_Average = (leg->FSR_Toe_Average + leg->FSR_Heel_Average);

  // TN 5/8/19

  leg->p_steps->curr_voltage_Toe = leg->FSR_Toe_Average;
  leg->p_steps->curr_voltage_Heel = leg->FSR_Heel_Average;

  if (FLAG_TOE_HEEL_SENSORS || FLAG_TOE_SENSOR)
  {
    if ((Flag_Ankle_Cfg == true)) {
      leg->p_steps->curr_voltage = leg->FSR_Toe_Average;
    }
    else if ((Flag_Knee_Cfg == true)) {
      leg->p_steps->curr_voltage = leg->FSR_Combined_Average;
    }
  }
  else {
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
    take_baseline(right_leg->state, right_leg->state_old, right_leg->p_steps, right_leg->p_FSR_baseline_FLAG);
  }
  if (left_leg->FSR_baseline_FLAG) {
    take_baseline(left_leg->state, left_leg->state_old, left_leg->p_steps, left_leg->p_FSR_baseline_FLAG);
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
    }

    // TN 5/15/19
    if (streamTimerCount == 1 && flag_auto_KF == 1) {
      Auto_KF_Knee(left_leg, Control_Mode);
      Auto_KF_Knee(right_leg, Control_Mode);
    }

    streamTimerCount++;

    pid(left_leg, left_leg->Average_Trq);
    pid(right_leg, right_leg->Average_Trq);

    pid_Knee(left_leg, left_leg->Average_Trq_Knee);  // TN 5/13/19
    pid_Knee(right_leg, right_leg->Average_Trq_Knee);  // TN 5/13/19


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

    Serial.println("right_leg->p_steps->plant_peak_mean");
    Serial.println(right_leg->p_steps->plant_peak_mean);
    Serial.println("right_leg->p_steps->plant_peak_mean_Toe");
    Serial.println(right_leg->p_steps->plant_peak_mean_Toe);
    Serial.println("right_leg->p_steps->plant_peak_mean_Heel");
    Serial.println(right_leg->p_steps->plant_peak_mean_Heel);
    Serial.println("right_leg->Setpoint_Ankle_Pctrl");
    Serial.println(right_leg->Setpoint_Ankle_Pctrl);
    Serial.println("right_leg->PID_Setpoint");
    Serial.println(right_leg->PID_Setpoint);
    Serial.println("right_leg->PID_Setpoint_Knee");
    Serial.println(right_leg->PID_Setpoint_Knee);
    Serial.println("right_leg->Setpoint_Knee_Pctrl");
    Serial.println(right_leg->Setpoint_Knee_Pctrl);
    Serial.println("right_leg->Setpoint_Ankle");
    Serial.println(right_leg->Setpoint_Ankle);
    Serial.println("right_leg->Setpoint_Knee");
    Serial.println(right_leg->Setpoint_Knee);
    
    




    Serial.println("FLAG_TOE_HEEL_SENSORS");
    Serial.println(FLAG_TOE_HEEL_SENSORS);

    Serial.println("Flag_Prop_Ctrl");
    Serial.println(Flag_Prop_Ctrl);
    Serial.println("flag_id");
    Serial.println(flag_id);
    Serial.println("flag_pivot");
    Serial.println(flag_pivot);
    Serial.println("Control Mode");
    Serial.println(Control_Mode);



    if ((left_leg->state == 3) && (left_leg->old_state == 1)) {
      left_leg->state_3_start_time = millis();
    }

    if ((left_leg->state == 1) && (left_leg->old_state == 3)) {
      left_leg->state_3_stop_time = millis();
    }

    if (left_leg->state_3_stop_time > left_leg->state_3_start_time) {
      left_leg->state_3_duration = left_leg->state_3_stop_time - left_leg->state_3_start_time;
    }

    left_leg->old_state = left_leg->state;

    if ((right_leg->state == 3) && (right_leg->old_state == 1)) {
      right_leg->state_3_start_time = millis();
    }

    else {

      if ((right_leg->state == 1) && (right_leg->old_state == 3)) {

        right_leg->state_3_stop_time = millis();


        if (right_leg->state_3_stop_time > right_leg->state_3_start_time) {
          right_leg->state_3_duration = right_leg->state_3_stop_time - right_leg->state_3_start_time;
        }
      }
    }

    right_leg->old_state = right_leg->state;

    if ((Control_Mode == 3 || Control_Mode == 4 ) && (abs(left_leg->Dorsi_Setpoint_Ankle) > 0 || abs(left_leg->Previous_Dorsi_Setpoint_Ankle) > 0) && left_leg->state == 1) { //GO 4/22/19
      left_leg->PID_Setpoint = left_leg->New_PID_Setpoint;   //Brute force the dorsiflexion set point to proportional control
    } else if ((Control_Mode == 3 || Control_Mode == 4 ) && (abs(right_leg->Dorsi_Setpoint_Ankle) > 0 || abs(right_leg->Previous_Dorsi_Setpoint_Ankle) > 0) && right_leg->state == 1) {
      right_leg->PID_Setpoint = right_leg->New_PID_Setpoint; //Brute force the dorsiflexion set point to proportional control
    } else {};

    // TN 5/15/19
    if ((Control_Mode == 3 || Control_Mode == 4 ) && (abs(left_leg->Dorsi_Setpoint_Knee) > 0 || abs(left_leg->Previous_Dorsi_Setpoint_Knee) > 0) && left_leg->state == 1) { //GO 4/22/19
      left_leg->PID_Setpoint_Knee = left_leg->New_PID_Setpoint_Knee;   //Brute force the dorsiflexion set point to proportional control
    } else if ((Control_Mode == 3 || Control_Mode == 4 ) && (abs(right_leg->Dorsi_Setpoint_Knee) > 0 || abs(right_leg->Previous_Dorsi_Setpoint_Knee) > 0) && right_leg->state == 1) {
      right_leg->PID_Setpoint_Knee = right_leg->New_PID_Setpoint_Knee; //Brute force the dorsiflexion set point to proportional control
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

    // TN 5/9/19

    left_leg->N3 = Control_Adjustment(left_leg, left_leg->state, left_leg->state_old, left_leg->p_steps,
                                      left_leg->N3, left_leg->New_PID_Setpoint, left_leg->p_Setpoint_Ankle_Pctrl,
                                      left_leg->New_PID_Setpoint_Knee, left_leg->p_Setpoint_Knee_Pctrl, Control_Mode, left_leg->Prop_Gain,
                                      left_leg->FSR_baseline_FLAG, &left_leg->FSR_Ratio, &left_leg->FSR_Ratio_Toe, &left_leg->FSR_Ratio_Heel,
                                      &left_leg->Max_FSR_Ratio, &left_leg->Max_FSR_Ratio_Toe, &left_leg->Max_FSR_Ratio_Heel);
    right_leg->N3 = Control_Adjustment(right_leg, right_leg->state, right_leg->state_old, right_leg->p_steps,
                                       right_leg->N3, right_leg->New_PID_Setpoint, right_leg->p_Setpoint_Ankle_Pctrl,
                                       right_leg->New_PID_Setpoint_Knee, right_leg->p_Setpoint_Knee_Pctrl, Control_Mode, right_leg->Prop_Gain,
                                       right_leg->FSR_baseline_FLAG, &right_leg->FSR_Ratio, &right_leg->FSR_Ratio_Toe, &right_leg->FSR_Ratio_Heel,
                                       &right_leg->Max_FSR_Ratio, &right_leg->Max_FSR_Ratio_Toe, &right_leg->Max_FSR_Ratio_Heel);

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
  leg->activate_in_3_steps = 1;
  leg->Previous_Setpoint_Ankle = 0;
  leg->Previous_Setpoint_Knee = 0;  // TN 5/8/19

  leg->coef_in_3_steps = 0;
  leg->num_3_steps = 0;

  leg->first_step = 1;
  counter_msgs = 0;
  leg->Heel_Strike_Count = 0;
  leg->score = 0;
  leg->Heel_Strike = 0;
  leg->NO_Biofeedback = true;
}
