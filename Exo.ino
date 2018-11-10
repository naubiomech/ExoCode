
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

#include <float.h>
#include <i2c_t3.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "System.hpp"
#include "Board.hpp"
#include "Receive_and_Transmit.hpp"
#include "TimerOne.h"
#include "Exoskeleton.hpp"

ExoSystem* exoSystem;
Exoskeleton* exo;

void setup() {
  exoSystem = setupBoard();
  exo = exoSystem->exo;

  // Fast torque calibration
  exo->calibrateTorque();

  digitalWrite(LED_PIN, HIGH);

  // set the interrupt
  Timer1.initialize(2000);         // initialize timer1, and set a 10 ms period *note this is 10k microseconds*
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

void callback() {

  resetMotorIfError();

  calculate_averages();

  check_FSR_calibration();

  check_Balance_Baseline();

  rotate_motor();

}

void loop() {

  if (exoSystem->trial->receiveDataTimer.check() == 1) {
    if (exoSystem->commandSerial->available() > 0) {
      receive_and_transmit(exoSystem);
    }

    exoSystem->trial->receiveDataTimer.reset();
  }

  if (exoSystem->trial->bluetoothStream != 1) {
    reset_starting_parameters();
  }
}

void resetMotorIfError() {

  if (exoSystem->trial->bluetoothStream == 1) {

    //TODO implement error checking

  }
}

void calculate_averages() {
  exo->measureSensors();
}

void check_FSR_calibration() {

  //TODO implement fsr calibration

}

void check_Balance_Baseline() {
  //TODO Implement the balance baseline
}

void rotate_motor() {

  if (exoSystem->trial->bluetoothStream == 1)
  {
    if (exoSystem->trial->reportDataTimer.check())
    {
      send_report(exoSystem->commandSerial);
      exoSystem->trial->reportDataTimer.reset();
    }

    //TODO apply auto kf here

    exo->applyTorque();

    exo->applyStateMachine();

    exo->setZeroIfSteadyState();

    exo->adjustControl();
  }
}

void reset_starting_parameters() {
  //Reset the starting values
  exo->resetStartingParameters();
}
