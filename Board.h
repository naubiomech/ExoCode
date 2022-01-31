#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER

#define DUAL_BOARD_REV8_1 1

#if BOARD_VERSION == DUAL_BOARD_REV8_1
#define ENABLE_PWM

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A2;
const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A3;

const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A0;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A6;

const unsigned int MOTOR_LEFT_ANKLE_PIN = 3;   
const unsigned int MOTOR_RIGHT_ANKLE_PIN = 10; 

const unsigned int RED = 12;
const unsigned int GREEN = 9;
const unsigned int BLUE = 8;

const unsigned int MOTOR_ENABLE_PIN = 5;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 4;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 11;

const unsigned int MOTOR_CURRENT_LEFT_ANKLE_PIN = A7; 
const unsigned int MOTOR_CURRENT_RIGHT_ANKLE_PIN = A1; 

const unsigned int PWR_SCL = A5;   //SCL
const unsigned int PWR_SDA = A4;   //SDA
#endif

#endif
