#ifndef SYSTEM_HEADER
#define SYSTEM_HEADER
#include <Metro.h> // Include the Metro library
#include <SoftwareSerial.h>

// ===== A Exo =====
Metro slowThisDown = Metro(1);  // Set the function to be called at no faster a rate than once per millisecond
Metro BnoControl = Metro(10);

//To interrupt and to schedule we take advantage of the
elapsedMillis timeElapsed;
double startTime = 0;
int streamTimerCount = 0;

int stream = 0;

char holdon[24];
char *holdOnPoint = &holdon[0];
char Peek = 'a';

SoftwareSerial bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);                  // Sets an object named bluetooth to act as a serial port

int Trq_time_volt = 0; // 1 for time 0 for volt 2 for proportional gain 3 for pivot proportional control
int Old_Trq_time_volt = Trq_time_volt;
int flag_13 = 1;
int flag_count = 0;

int flag_semaphore = 0;

volatile double motor_driver_count_err;

double start_time_callback, start_time_timer;

int time_err_motor;
int time_err_motor_reboot;

int flag_enable_catch_error = 1;

bool motor_error = false;

// ===== Auto KF =====
int flag_auto_KF = 0;

// ===== IMU =====
sensors_event_t event;
imu::Vector<3> euler;

volatile double stability_trq;

Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, 1, BNO055_ADDRESS_A, I2C_MASTER, IMU_1_PINS, I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);

bool IMU_flag;

// ===== Leg =====
Leg left_leg_value = Leg();
Leg right_leg_value = Leg();
Leg* left_leg = &left_leg_value;
Leg* right_leg = &right_leg_value;

// ===== Msg Functions =====
msg val_msg_send;
msg val_msg_receive;
msg* p_msg_send = &val_msg_send;
msg* p_msg_receive = &val_msg_receive;

double data_to_send[8];
double *data_to_send_point = &data_to_send[0];

// ===== Torque Speed Adj =====
steps val_L;
steps val_R;

#endif
