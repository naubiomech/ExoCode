//Robert Libby, December 18, 2017.  NAU Biomechatronics
//This Code gives assistance to a user to help them during the "toe-off" portion of the gait cycle
//In order for this code to run correctly, in its entirety, it must interface with the MATLAB GUI I created, though it can be chopped up through comments and be functional
// http://ww1.microchip.com/downloads/en/DeviceDoc/21897a.pdf

#include <elapsedMillis.h>
#include <EEPROM.h>
elapsedMillis timeElapsed;

int address_torque = 0;
int address_FSR = 9;
boolean sigm_done = true;
//boolean filter_done = true;

double Ts = 0.001;
double exp_mult = 1500.0;
double New_PID_Setpoint = 0.0;
double Old_PID_Setpoint = 0.0;
boolean sigm_flag = true;
//boolean filter_flag = not(sigm_flag);
int change = 1;
int just_first_time = 1;
int n_iter , N_step;

double array_ref[3];
double array_out[3];
double *p_array_ref = array_ref;
double *p_array_out = array_out;
double a_vect[3] = {1.0, -1.864734299516908, 0.869308501948704};
double *p_a_vect = a_vect;
double b_vect[3] = {0.001143550607949, 0.002287101215898, 0.001143550607949};
double *p_b_vect = b_vect;

int clean_flag = 1;

double Tcal_LL = 0;                                                     //Initialize the variable for the calibration of the torque sensor
double T_act_LL = 0;                                                       //The value of torque on the LL,

double startTime = 0;
int streamTimerCount = 0;
double fsrLongCurrent = 0;
double fsrShortCurrent = 0;
const unsigned int fsr_sense_Long = 2;
const unsigned int fsr_sense_Short = 3;
double doubleFSR = 0;
int intFSR = 0;
double fsr_cal_Long = 0;
double fsr_cal_Short = 0;
double fsr_thresh_long = .8;
double fsr_thresh = .8;

int flag = 0;
float filterVal = .8;
double smoothedVal = 0;

int Vol_LL;
const unsigned int motor_LL = A14;                                     //analogwrite pin
double kp = 300;
double ki = 0;
double kd = 0;

int count = 0;
int stream = 0;
int i = 0;
int j = 0;


int dim = 3;
double Tarray[3] = {0};
double *TarrayPoint = &Tarray[0];
double Average = 0;

char holdon[24];
char *holdOnPoint = &holdon[0];
char Peek = 'a';
int garbage = 0;
int R_state = 1;

int streamCount = 0;

const unsigned int onoff = 2;                                          //The digital pin connected to the motor on/off swich
const unsigned int zero = 1540;                                       //whatever the zero value is for the PID analogwrite setup
const unsigned int which_leg_pin = 15;


#include <PID_v1.h>                                                  //Includes the PID library so we can utilize PID control
int PID_sample_time = 1;                                             //PID operates at 1000Hz, calling at a freq of 1 ms.
double PID_Setpoint, Input_LL, Output_LL;                             //Initializes the parameters for the PID controll
PID PID_LL(&Input_LL, &Output_LL, &PID_Setpoint, kp, ki, kd, DIRECT); //Sets up PID for the right leg

double Setpoint_Ankle = 0;
double Setpoint_earlyStance = .25 * Setpoint_Ankle;
#include "TimerOne.h"

#include <SoftwareSerial.h>                                          //Includes the SoftwareSerial library to be able to use the bluetooth Serial Communication
int bluetoothTx = 0;                                                 // TX-O pin of bluetooth mate, Teensy D0
int bluetoothRx = 1;                                                 // RX-I pin of bluetooth mate, Teensy D1
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);                  // Sets an object named bluetooth to act as a serial port

void setup()
{

  Timer1.initialize(1000);         // initialize timer1, and set a 10 ms period *note this is 10k microseconds*
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  bluetooth.begin(115200);
  Serial.begin(115200);                                               //Starts Serial communication at 115k to match the default bluetooth setting

  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  pinMode(which_leg_pin, INPUT);
  pinMode(13, OUTPUT);
  pinMode(onoff, OUTPUT);                                             //setup for digital on off for motor control board
  digitalWrite(onoff, LOW);                                           //Ensures Motor does not start pre-maturely
  //Sets up the pin that is meant to read in torq (A0) as the input
  pinMode(A0, INPUT);

  int PID_sample_time = 1;                                            //PID operates at 1000Hz, calling at a freq of 1 ms.
  PID_Setpoint = 0;
  Setpoint_Ankle = whichLeg(which_leg_pin, Setpoint_Ankle);
  Setpoint_earlyStance = .25 * Setpoint_Ankle;


  analogWrite(motor_LL, zero);
  PID_LL.SetMode(AUTOMATIC);
  PID_LL.SetTunings(kp, ki, kd);                                      //Kp, Ki, Kd ##COULD BE AUTOTUNED
  PID_LL.SetOutputLimits(-1500, 1500);                                  //range of Output around 0 ~ 1995 ##THIS IS DIFFERENT NOW AND SHOULD CONCRETELY CONFIRM
  PID_LL.SetSampleTime(PID_sample_time);                              //what is the sample time we want in millis

  double  tcal_time = millis();                                       //ignores the time it took to execute all previous code when calibratin
  int  count = 0;                                                    //Starts count for calibration at 0
  digitalWrite(13, HIGH);
  while (millis() - tcal_time < 1000)
  { //Calibrates the LL for a total time of 1 second,
    Tcal_LL += analogRead(A4);                                        //Sums the torque read in and sums it with all previous red values
    count ++;                                                         //Increments count
  }
  Tcal_LL = (Tcal_LL / count) * (3.3 / 4096);

  tcal_time = millis();
  while (millis() - tcal_time < 5000)
  {
    fsrLongCurrent = fsr(fsr_sense_Long);
    fsrShortCurrent = fsr(fsr_sense_Short);
    if (fsrLongCurrent > fsr_cal_Long)
    {
      fsr_cal_Long = fsrLongCurrent;
    }
    if ( fsrShortCurrent > fsr_cal_Short)
    {
      fsr_cal_Short = fsrShortCurrent;
    }
  }
  digitalWrite(13, LOW);
  //  bluetooth.println("*");
  //  bluetooth.println("u");
  //  bluetooth.println("i");
  //  bluetooth.println("o");
}

void callback()
{
  if ((stream == 1))
  {
    if (streamTimerCount == 10)
    {
      bluetooth.println(Average);
      bluetooth.println(R_state);//
      streamTimerCount = 0;
    }
    streamTimerCount++;

    for (int j = dim; j >= 0; j--)                    //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
    { // there are the number of spaces in the memory space minus 2 actions taht need to be taken
      *(TarrayPoint + j) = *(TarrayPoint + j - 1);                //Puts the element in the following memory space into the current memory space
    }
    T_act_LL = -get_LL_torq();
    *(TarrayPoint) = T_act_LL;
    int i = 0;
    Average = 0;
    for (i = 0; i < dim; i++)
    {
      Average = Average + *(TarrayPoint + i);
    }
    Average = Average / dim;

    pid(Average);
  }
}

void loop()
{
  /***************Ankle CODE********************/
  while (bluetooth.available() == 0) {
    //    stream=1;
    if ((stream == 1))                                   //Waits to give Assistance until the Start button has been pushed on the MATLAB GUI
    {
      switch (R_state)
      {
        case 1: //Swing
          if ((fsr(fsr_sense_Long) > (fsr_thresh_long * fsr_cal_Long)) && (fsr(fsr_sense_Short) < (fsr_thresh * fsr_cal_Short)))
          {
            digitalWrite(13, LOW);
            Old_PID_Setpoint = PID_Setpoint;
            New_PID_Setpoint = Setpoint_earlyStance;
            R_state = 2;
          }
          if ((fsr(fsr_sense_Long) > ((fsr_thresh_long) * fsr_cal_Long) && (fsr(fsr_sense_Short) > fsr_thresh * fsr_cal_Short)))
          {
            digitalWrite(13, HIGH);
            Old_PID_Setpoint = PID_Setpoint;
            New_PID_Setpoint = Setpoint_Ankle;
            R_state = 3;
          }
          break;
        case 2: // early Stance
          if ((fsr(fsr_sense_Long) > (fsr_thresh_long * fsr_cal_Long) && (fsr(fsr_sense_Short) > fsr_thresh * fsr_cal_Short)))
          {
            digitalWrite(13, HIGH);
            Old_PID_Setpoint = PID_Setpoint;
            New_PID_Setpoint = Setpoint_Ankle;
            R_state = 3;
          }
          if ((fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)) && (fsr(fsr_sense_Short) < (fsr_thresh * fsr_cal_Short)))
          {
            digitalWrite(13, LOW);
            Old_PID_Setpoint = PID_Setpoint;
            New_PID_Setpoint = 0;
            //            PID_Setpoint = 0;//-3;  //Dorsiflexion for MAriah in Swing, otherwise should be 0
            R_state = 1;
          }
          break;
        case 3: //Late Stance
          if ((fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)) && (fsr(fsr_sense_Short) < (fsr_thresh * fsr_cal_Short)))
          {
            digitalWrite(13, LOW);
            Old_PID_Setpoint = PID_Setpoint;
            New_PID_Setpoint = 0;
            //            PID_Setpoint = 0;//-3; //Dorsiflexion for MAriah in Swing, otherwise should be 0
            R_state = 1;
          }
          break;
      }


      if (sigm_flag) {
        if ((abs(New_PID_Setpoint - PID_Setpoint) > 0.5) && (sigm_done)) {
          //if (1) {
          sigm_done = false;
          n_iter = 0;
          N_step = round((1 / (Ts * exp_mult)) * 10);

          if ((N_step % 2)) {
            N_step++;
          }
        }

        if (n_iter < N_step) {

          PID_Setpoint = Change_PID_Setpoint_Sigm(New_PID_Setpoint, PID_Setpoint, Old_PID_Setpoint, Ts, exp_mult, n_iter, N_step);
          n_iter++;
        }
        if (n_iter >= N_step) {
          sigm_done = true;
        }
      }// end if sigm

      Serial.print(R_state);
      Serial.print(",");
      Serial.print(New_PID_Setpoint);
      Serial.print(",");
      Serial.println(PID_Setpoint);
      delay(15);



    }// end stream
  }// end bluetooth
  //====================================================================================================================
  //====================================================================================================================
  //====================================================================================================================
  //====================================================================================================================

  Peek = bluetooth.peek();
  if (Peek == 'D')                               //if MATLAB sent the character D
  {
    garbage = bluetooth.read();
    sendVals();                                 //MATLAB is expecting to recieve the Torque Parameters
  }
  if (Peek == 'F')                                //If MATLAB sent the character F
  { //MATLAB wants to write a new Torque Value
    garbage = bluetooth.read();
    delay(10);
    recieveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&Setpoint_Ankle, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Setpoint_Ankle = whichLeg(which_leg_pin, Setpoint_Ankle); //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
    Setpoint_earlyStance = .25 * Setpoint_Ankle;              //Recieved the large data chunk chopped into bytes, a roundabout way was needed
  }
  if (Peek  == 'E')                               //If MATLAB sent the character E
  {
    garbage = bluetooth.read();
    digitalWrite(onoff, HIGH);                    //The GUI user is ready to start the trial so Motor is enabled
    bluetooth.println(0);
    bluetooth.println(0);
    stream = 1;                                   //and the torque data is allowed to be streamed
    streamTimerCount = 0;
    digitalWrite(13, HIGH);
  }
  if (Peek == 'G')                                //If MATLAB sent the character G
  {
    garbage = bluetooth.read();
    digitalWrite(onoff, LOW);                     //The GUI user is ready to end the trial, so motor is disabled
    stream = 0;                                   //and the torque data is no longer allowed to be streamed.
    digitalWrite(13, LOW);                        //*Note* Even though no new data is being streamed the buffer in MATLAB still may contain data that has not been read in yet
    //Or data is still "In transit" that needs to be accounted for on the MATLAB side or the may will start doing Wierd things
  }
  if (Peek == 'H')                                 //If MATLAB sent the character H
  {
    garbage = bluetooth.read();
    digitalWrite(13, LOW);                         //The user Wants to Recalibrate all the Sensors
    double  tcal_time = millis();                  //ignores the time it took to execute all previous code when calibrating
    int  count = 0;                                //Starts count for calibration at 0
    while (millis() - tcal_time < 1000)
    { //Calibrates the LL for a total time of 1 second,
      Tcal_LL += analogRead(A4);                   //Sums the torque read in and sums it with all previous red values
      count++;                                     //Increments count
    }
    Tcal_LL = (Tcal_LL / count) * (3.3 / 4096);    // Averages torque over a second

    write_torque_bias(address_torque, Tcal_LL);

  }
  if (Peek == 'K')                                 //If MATLAB sent the character K, then it wants to know what the current PID Parameters are
  {
    garbage = bluetooth.read();
    bluetooth.println(kp);                         //Sends the PID parameters
    bluetooth.println(kd);
    bluetooth.println(ki);
  }
  if (Peek == 'L')
  {
    garbage = bluetooth.read();
    fsr_cal_Short = 0;
    fsr_cal_Long = 0;
    startTime = millis();
    while (millis() - startTime < 5000)
    {
      fsrLongCurrent = fsr(fsr_sense_Long);
      fsrShortCurrent = fsr(fsr_sense_Short);
      if (fsrLongCurrent > fsr_cal_Long)
      {
        fsr_cal_Long = fsrLongCurrent;
      }
      if ( fsrShortCurrent > fsr_cal_Short)
      {
        fsr_cal_Short = fsrShortCurrent;
      }
    }
    write_FSR_values(address_FSR, fsr_cal_Long);

    write_FSR_values((address_FSR + sizeof(double) + sizeof(char)), fsr_cal_Short);
  }
  if (Peek == 'M')                                 //If MATLAB Sent the character M, then it wants to write new PID parameters
  {
    garbage = bluetooth.read();
    recieveVals(24);                               //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
    //MATLAB Sent Kp, then Kd, then Ki.
    memcpy(&kp, holdOnPoint, 8);                   //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    memcpy(&kd, holdOnPoint + 8, 8);               //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
    memcpy(&ki, holdOnPoint + 16, 8);              //Recieved the large data chunk chopped into bytes, a roundabout way was needed
    PID_LL.SetTunings(kp, ki, kd);
  }
  if (Peek == 'N')                                 //If MATLAB sent the character N, it wants to check that the Arduino is behaving
  {
    garbage = bluetooth.read();
    bluetooth.println("B");                        //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
  }
  //  if (Peek == 'Z') {
  //    //    Check the bluetooth before actually activate the motors
  //    garbage = bluetooth.read();
  //    bluetooth.println('Z');

  //  }
  //===============================================================================================================
  //===============================================================================================================
  //===============================================================================================================
  //  Serial.println(Peek);
  //  Serial.println("*");
  //  Serial.println("*");
  //  Serial.println("*");


  if (Peek == '<') {
    // Check Memory
    //    clean_flag = 1;
    garbage = bluetooth.read();
    if (check_torque_bias(address_torque)) {
      //      garbage = bluetooth.read();
      bluetooth.println('y');
      Serial.println("Torque y");
    } else {
      //      garbage = bluetooth.read();
      bluetooth.println('*');
      Serial.println("Torque *");
    }
    //  read_torque_bias(int address_torque_l)
    if ((check_FSR_values(address_FSR)) && (check_FSR_values(address_FSR + sizeof(double) + 1))) {
      //      garbage = bluetooth.read();
      bluetooth.println('y');
      Serial.println("FSR y");
    } else {
      //      garbage = bluetooth.read();
      bluetooth.println('*');
      Serial.println("FSR *");
    }
    Peek = 'a';
  }// end if Check Memory


  if (Peek == '>')
  { // Clean Memory
    Serial.println("Enter Clear ");
    if (clean_torque_bias(address_torque)) {
      Serial.println("Clear Torque ");
    } else {
      Serial.println("No clear Torque");
    }
    if (clean_FSR_values(address_FSR)) {
      Serial.println("Clear FSR ");
    } else {
      Serial.println("No clear FSR");
    }
    if (clean_FSR_values(address_FSR + sizeof(double) + sizeof(char))) {
      Serial.println("Clear FSR ");
    } else {
      Serial.println("No clear FSR");
    }
    garbage = bluetooth.read();
    Serial.println(Peek);
    Peek = 'a';
    Serial.println(Peek);
    Serial.println("Exit Clear");
  }// end if clean memory



  Peek = 0;
}





