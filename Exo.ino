
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

#include <cstddef>
#include <float.h>
#include <i2c_t3.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>
#include "BoardBuilder.hpp"
#include "Board.hpp"
#include "TimerOne.h"
#include "Exoskeleton.hpp"
#include "ExoBuilder.hpp"
#include "Linked_List.hpp"


Exoskeleton* exo;

void setup() {
  exo = setupSystem();
  Serial.println("Got exo");
  exo->startTrial();

  // set the interrupt
  Timer1.initialize(2000);         // initialize timer1, and set a 10 ms period *note this is 10k microseconds*
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

void callback() {
  exo->run();
}

void loop() {

  exo->sendReport();
  exo->receiveMessages();
  exo->checkReset();

}

Exoskeleton* setupSystem(){
  Serial.begin(115200);
  delay(500);
  Board* board = QuadBoardDirector().build();
  Exoskeleton* exo = QuadExoDirector().build(board);
  delete board;
  return exo;
}
