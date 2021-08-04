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

#define CONTROL_LOOP_HZ           500
#define CONTROL_TIME_STEP         1 / CONTROL_LOOP_HZ
#define COMMS_LOOP_HZ             50    

char cmd_from_Gui = 0;
bool stream{false};
byte holdon[96];
byte *holdOnPoint = &holdon[0];

#include <ArduinoBLE.h>
#include <mbed.h>
#include <rtos.h>

#include "Board.h"
#include "Msg_functions.h"
#include "Msg_functions.h"
#include "ema_filter.h"
//----------------------------------------------------------------------------------

inline double sample_thermocouple(unsigned int analogPin) {
  double voltage = analogRead(analogPin)*3.3/4096.0;
  return (voltage - 1.25) / 0.005;
}


// Initialize the system
void setup()
{
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
  digitalWrite(GREEN, HIGH);
  
  analogWriteResolution(12);
  analogReadResolution(12);
  
  Serial.begin(1000000);
  delay(100);

  setupBLE();
}


//----------------------------------------------------------------------------------
void loop()
{  
  //Required for HCI communications
  BLE.poll();
  
  if (stream) {
    double temp_deg_F = sample_thermocouple(FSR_SENSE_LEFT_TOE_PIN);
    Serial.println(temp_deg_F);
    send_thermo_message(temp_deg_F);
  }
  
  rtos::ThisThread::sleep_for(1000 / COMMS_LOOP_HZ);
}
