#ifndef SYSTEM_HEADER
#define SYSTEM_HEADER
#include <Metro.h> // Include the Metro library
#include <SoftwareSerial.h>
#include "Exoskeleton.h"
#include "IMU.h"

struct Trial {
  int bluetoothStream = 0;
  elapsedMillis timeElapsed;
  int streamTimerCount = 0;
};
// ===== A Exo =====
Metro slowThisDown = Metro(1);  // Set the function to be called at no faster a rate than once per millisecond
Metro BnoControl = Metro(10);

//To interrupt and to schedule we take advantage of the
elapsedMillis timeElapsed;
double fsrCalibrationStartTime = 0;
int streamTimerCount = 0;

int stream = 0;

char holdon[24];
char *holdOnPoint = &holdon[0];

SoftwareSerial bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);                  // Sets an object named bluetooth to act as a serial port

int Trq_time_volt = 0; // 1 for time 0 for volt 2 for proportional gain 3 for pivot proportional control
int Old_Trq_time_volt = Trq_time_volt;

volatile double motor_driver_count_err;

int time_err_motor;
int time_err_motor_reboot;

int flag_enable_catch_error = 1;

// ===== FSR Parameters =====
int FSR_FIRST_Cycle = 1;
int FSR_CAL_FLAG = 0;

double base_1, base_2;

// ===== Receive_and_Transmit
bool FLAG_BALANCE = false;
bool FLAG_BALANCE_BASELINE = false;

// ===== Auto KF =====
int flag_auto_KF = 0;

// ===== IMU =====
imu::Vector<3> euler;

volatile double stability_trq;

Adafruit_BNO055 bno;

// ===== Leg =====
Exoskeleton* exo;

// ===== Msg Functions =====
double data_to_send[8];
double *data_to_send_point = &data_to_send[0];

#endif
