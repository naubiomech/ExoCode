#include <elapsedMillis.h>
#include <EEPROM.h>
#include "TimerOne.h"
#include <PID_v2.h>
#include <SoftwareSerial.h>

typedef struct {
  int    n_steps;
  int    n_kf;
  int n_cycles;
  int prev_state;
  double dorsi_time;
  double plant_time;
  double dorsi_time_average;
  double four_step_dorsi_time[4];
  double four_step_plant_time[4];
  bool flag_take_average = false;
  bool flag_N3_adjustment_time = false;
  double dorsi_mean;
  double plant_mean;
  double dorsi_mean_old;
  double plant_mean_old;
  int count_steps;
  double torque_val;
  double torque_val_average;
  bool flag_1_step = false;
  bool flag_take_baseline = false;
  bool torque_adj = false;
  double Setpoint;
  double plant_mean_base;
  double dorsi_mean_base;
  double voltage_ref;
  double curr_voltage;
  double perc_l = 0.5;

} steps;

steps val_L;
steps val_R;
steps* L_p_steps = &val_L;
steps* R_p_steps = &val_R;


//To store in memory
int address_torque_LL = 0;
int address_torque_RL = 9;
int address_FSR_LL = 18;
int address_FSR_RL = 36;

//To shape the reference
double Ts = 0.001;
double exp_mult_RL = 1500.0;
boolean sigm_flag_RL = true;
double exp_mult_LL = 1500.0;
boolean sigm_flag_LL = true;

double New_PID_Setpoint_RL = 0.0;
double Old_PID_Setpoint_RL = 0.0;
double New_PID_Setpoint_LL = 0.0;
double Old_PID_Setpoint_LL = 0.0;

double N3 = 500;
double N2 = 4;
double N1 = 4;

double N3_LL = N3;
double N2_LL = N2;
double N1_LL = N1;

double N3_RL = N3;
double N2_RL = N2;
double N1_RL = N1;


long sig_time_RL = 0;
long sig_time_old_RL = 0;
long sig_time_LL = 0;
long sig_time_old_LL = 0;

//boolean filter_flag = not(sigm_flag);
int change = 1;
int just_first_time = 1;
int n_iter_RL, N_step_RL;
int n_iter_LL, N_step_LL;

boolean sigm_done_LL = true;
boolean sigm_done_RL = true;

//To filter the reference
double array_ref[3];
double array_out[3];
double *p_array_ref = array_ref;
double *p_array_out = array_out;
double a_vect[3] = {1.0, -1.864734299516908, 0.869308501948704};
double *p_a_vect = a_vect;
double b_vect[3] = {0.001143550607949, 0.002287101215898, 0.001143550607949};
double *p_b_vect = b_vect;
int flag = 0;
float filterVal = .8;
double smoothedVal = 0;
int clean_flag = 1;
//boolean filter_done = true;

//To interrupt and to schedule
elapsedMillis timeElapsed;
double startTime = 0;
int streamTimerCount = 0;

// To set torque bias
double Tcal_LL = 0;
double Tcal_RL = 0;
int count = 0;

// To set FSR bias and to identify the states
double fsrLongCurrent = 0;
double fsrShortCurrent = 0;
//const unsigned int fsr_sense_Long = 2;
//const unsigned int fsr_sense_Short = 3;
//const unsigned int fsr_sense_Left_Heel = A12;
//const unsigned int fsr_sense_Left_Toe = A13;
//const unsigned int fsr_sense_Right_Heel = A14;
//const unsigned int fsr_sense_Right_Toe = A15;

const unsigned int fsr_sense_Left_Heel = A14;
const unsigned int fsr_sense_Left_Toe = A15;
const unsigned int fsr_sense_Right_Heel = A12;
const unsigned int fsr_sense_Right_Toe = A13;

double fsr_Left_Heel = 0;
double fsr_Left_Toe = 0;
double fsr_Right_Heel = 0;
double fsr_Right_Toe = 0;
double fsr_Left_Heel_thresh = 0;                      //These "thresh" values are what I called the "cal" values in previous versions
double fsr_Left_Toe_thresh = 0;                       //Also known asthe values to determine when to transition states
double fsr_Right_Heel_thresh = 0;
double fsr_Right_Toe_thresh = 0;

double doubleFSR = 0;
int intFSR = 0;

double fsr_percent_thresh_Left_Heel = .9;              //These are the percentage of the threshold values where the code will say
double fsr_percent_thresh_Left_Toe = .9;               //if it is "this" percent of our calibration/threshold value we are "close enough" to change the state
double fsr_percent_thresh_Right_Heel = .9;             //These are the percentage of the threshold values where the code will say
double fsr_percent_thresh_Right_Toe = .9;              //if it is "this" percent of our calibration/threshold value we are "close enough" to change the state

// For the PID
//Left Motor control Pin = A21
//Right Motor control pin = A22
//Enable pin for both = D22

double T_act_LL, T_act_RL;
int Vol_LL, Vol_RL;
const unsigned int motor_LL_pin = A21;                                     //analogwrite pin
const unsigned int motor_RL_pin = A22;                                     //analogwrite pin
//const unsigned int enable_motors = D22;                                     //digital pin

double kp_LL = 800;
double ki_LL = 0;
double kd_LL = 0;
double KF_LL = 1;
double kp_RL = 800;
double ki_RL = 0;
double kd_RL = 0;
double KF_RL = 1;

//int count = 0;
int stream = 0;
int i = 0;
int j = 0;


int dim = 3;
double Tarray_LL[3] = {0};
double *TarrayPoint_LL = &Tarray_LL[0];
double Average_LL = 0;
double Tarray_RL[3] = {0};
double *TarrayPoint_RL = &Tarray_RL[0];
double Average_RL = 0;


char holdon[24];
char *holdOnPoint = &holdon[0];
char Peek = 'a';
int garbage = 0;

int R_state = 1;
int L_state = 1;
int R_state_old = 1;
int L_state_old = 1;
int state_count_RL_13 = 0;                                            //These values are included to count how many times the the transition between states occurs
int state_count_RL_31 = 0;                                            //The intent is to not let the code to transition states if an outlier sensor measurement were to occur
int state_count_LL_13 = 0;
int state_count_LL_31 = 0;

int streamCount = 0;

const unsigned int onoff = 2;                                          //The digital pin connected to the motor on/off swich
const unsigned int zero = 1540;                                       //whatever the zero value is for the PID analogwrite setup
const unsigned int which_leg_pin = 15;


//Includes the PID library so we can utilize PID control
int PID_sample_time = 1;                                             //PID operates at 1000Hz, calling at a freq of 1 ms.
double PID_Setpoint_LL, Input_LL, Output_LL;                             //Initializes the parameters for the PID controll
double PID_Setpoint_RL, Input_RL, Output_RL;                             //Initializes the parameters for the PID controll
PID PID_LL(&Input_LL, &Output_LL, &PID_Setpoint_LL, kp_LL, ki_LL, kd_LL, DIRECT); //Sets up PID for the right leg
PID PID_RL(&Input_RL, &Output_RL, &PID_Setpoint_RL, kp_RL, ki_RL, kd_RL, DIRECT); //Sets up PID for the right leg

double Setpoint_Ankle_RL = 0;
double Setpoint_Ankle_LL = 0;

double*p_Setpoint_Ankle_LL = &Setpoint_Ankle_LL;
double*p_Setpoint_Ankle_RL = &Setpoint_Ankle_RL;

//double Setpoint_earlyStance = .25 * Setpoint_Ankle;              //No Longer needed, as we only have states 1 and 3


//Timer t_ref;

//Includes the SoftwareSerial library to be able to use the bluetooth Serial Communication
int bluetoothTx = 0;                                                 // TX-O pin of bluetooth mate, Teensy D0
int bluetoothRx = 1;                                                 // RX-I pin of bluetooth mate, Teensy D1
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);                  // Sets an object named bluetooth to act as a serial port



double R_state_3_start_time = 0;
double R_state_1_start_time = 0;
double R_start_from_1 = 0;
double R_start_from_3 = 0;

double L_state_3_start_time = 0;
double L_state_1_start_time = 0;
double L_start_from_1 = 0;
double L_start_from_3 = 0;


double state_counter_th=8;


//---------------------


void setup()
{
  // set the interrupt
  Timer1.initialize(2000);         // initialize timer1, and set a 10 ms period *note this is 10k microseconds*
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  // enable bluetooth
  bluetooth.begin(115200);
  Serial.begin(115200);
  //while (!Serial) {};

  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  // The led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // set pin mode for left and right sides
  pinMode(22, OUTPUT); //Enable disable the motors
  digitalWrite(22, LOW);

  pinMode(A19, INPUT); //enable the torque reading of the left torque sensor
  pinMode(A18, INPUT); //enable the torque reading of the right torque sensor

  // set PID
  int PID_sample_time = 1;                                            //PID operates at 1000Hz, calling at a freq of 1 ms.
  double PID_Setpoint_LL = 0;
  double Setpoint_Ankle_LL = 0;
  double Setpoint_earlyStance_LL = .25 * Setpoint_Ankle_LL;
  double PID_Setpoint_RL = 0;
  double Setpoint_Ankle_RL = 0;
  double Setpoint_earlyStance_RL = .25 * Setpoint_Ankle_RL;

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
  //  fast_FSR_calibration();

}

double L_stateTimerCount;
double L_flag_1 = 0;
double L_time_old_state;
double R_stateTimerCount;
double R_flag_1 = 0;
double R_time_old_state;

double R_activate_in_3_steps = 0;
double R_1st_step = 1;
double R_coef_in_3_steps = 0;
double R_start_step = 0;
double R_num_3_steps = 0;
double store_3sec_N1_RL = N1_RL;

double L_activate_in_3_steps = 0;
double L_1st_step = 1;
double L_coef_in_3_steps = 0;
double L_start_step = 0;
double L_num_3_steps = 0;


double L_store_N1 = 0;
double L_set_2_zero = 0;
double R_store_N1 = 0;
double R_set_2_zero = 0;

double One_time_L_set_2_zero = 1;
double One_time_R_set_2_zero = 1;


void callback()
{

  if ((stream == 1))
  {
    if (L_flag_1 == 0) {
      L_flag_1 = 1;
      L_time_old_state = L_state;
    }
    if (L_state != L_time_old_state) {
      L_flag_1 = 0;
      L_stateTimerCount = 0;
    } else {
      if (L_stateTimerCount >= 3 / 0.002) {
        //        Serial.println("Too Long");
        if (L_store_N1 == 0) {
          Serial.println("Change N1");
          L_set_2_zero = 1;
          L_store_N1 = 1;
        }

      } else {
        L_stateTimerCount++;
        if (L_store_N1) {
          L_set_2_zero = 0;
          L_store_N1 = 0;
        }

        //        set_2_zero = 0;

      }
    }

    if (R_flag_1 == 0) {
      R_flag_1 = 1;
      R_time_old_state = R_state;
    }
    if (R_state != R_time_old_state) {
      R_flag_1 = 0;
      R_stateTimerCount = 0;
    } else {
      if (R_stateTimerCount >= 3 / 0.002) {
        if (R_store_N1 == 0) {
          Serial.println("Change N1");
          R_set_2_zero = 1;
          R_store_N1 = 1;
        }
      } else {
        R_stateTimerCount++;
        if (R_store_N1) {
          R_set_2_zero = 0;
          R_store_N1 = 0;
        }
      }
    }

    //    Serial.println("LEFT");
    N3_LL = Adj_N3_speed_with_voltage_every_step(L_state, L_state_old, L_p_steps, N3_LL, New_PID_Setpoint_LL, p_Setpoint_Ankle_LL);
    //    Serial.println("RIGHT");
    N3_RL = Adj_N3_speed_with_voltage_every_step(R_state, R_state_old, R_p_steps, N3_RL, New_PID_Setpoint_RL, p_Setpoint_Ankle_RL);

    if (streamTimerCount == 5)
    {
      //Bluetooth print here
      //      Serial.print( (analogRead(A19) * (3.3 / 4096)));
      //      Serial.print(",");
      //      Serial.print((analogRead(A19) * (3.3 / 4096)));
      //      Serial.print(",");
      //      Serial.print(Tcal_RL);
      //      Serial.print(",");
      //      Serial.println(Tcal_LL);
      bluetooth.println(Average_RL);
      bluetooth.println(R_state);
      bluetooth.println(PID_Setpoint_RL);         //Right Setpoint
      bluetooth.println(fsr(fsr_sense_Right_Toe));         //Right Voltage/Voltage of FSR
      bluetooth.println(Average_LL);
      bluetooth.println(L_state);
      bluetooth.println(PID_Setpoint_LL);         //Left 9
      bluetooth.println(fsr(fsr_sense_Left_Toe));         //Left Voltage/Voltage of FSR
      streamTimerCount = 0;

      L_p_steps->curr_voltage = fsr(fsr_sense_Left_Toe);
      R_p_steps->curr_voltage = fsr(fsr_sense_Right_Toe);
      //Serial.println(fsr(fsr_sense_Left_Toe));
      //      Serial.println(" ");
      //      Serial.println(fsr(fsr_sense_Left_Toe));
      //      Serial.println(fsr_percent_thresh_Left_Toe * fsr_Left_Toe_thresh);
      //      Serial.print("Check if true ");
      //      Serial.println(((fsr(fsr_sense_Left_Toe) > fsr_percent_thresh_Left_Toe * fsr_Left_Toe_thresh)));
      //      Serial.println(L_state);

    }
    streamTimerCount++;
    for (int j = dim; j >= 0; j--)                    //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
    { // there are the number of spaces in the memory space minus 2 actions that need to be taken
      *(TarrayPoint_LL + j) = *(TarrayPoint_LL + j - 1);                //Puts the element in the following memory space into the current memory space
      *(TarrayPoint_RL + j) = *(TarrayPoint_RL + j - 1);
    }
    T_act_LL = get_LL_torq();
    T_act_RL = get_RL_torq();
    *(TarrayPoint_LL) = T_act_LL;
    *(TarrayPoint_RL) = T_act_RL;
    int i = 0;
    Average_LL = 0;
    Average_RL = 0;
    for (i = 0; i < dim; i++)
    {
      Average_LL = Average_LL + *(TarrayPoint_LL + i);
      Average_RL = Average_RL + *(TarrayPoint_RL + i);
    }
    Average_LL = Average_LL / dim;
    Average_RL = Average_RL / dim;
    pid(Average_LL, 1);
    pid(Average_RL, 2);
  }
}


void loop()
{

  while (bluetooth.available() == 0)
  {
    if (stream == 1)
    {
      state_machine_LL();  //for LL
      state_machine_RL();  //for RL
    }
    else
    {
      L_p_steps->count_steps = 0;
      L_p_steps->n_steps = 0;
      L_p_steps->flag_1_step = false;
      L_p_steps->flag_take_average = false;
      L_p_steps->flag_N3_adjustment_time = false;
      L_p_steps->flag_take_baseline = false;
      L_p_steps->torque_adj = false;

      R_p_steps->count_steps = 0;
      R_p_steps->n_steps = 0;
      R_p_steps->flag_1_step = false;
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

    }
  }

  receive_and_transmit();
}
