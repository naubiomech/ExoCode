#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER

// Danny, define board version


#define TMOTOR_REV1 5
#define DUAL_BOARD_REV6 4
#define DUAL_BOARD_REV6_1 3
#define QUAD_BOARD 2
#define DUAL_BOARD_REV4 1

#if BOARD_VERSION == DUAL_BOARD_REV6
#define ENABLE_PWM

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A2;
const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A3;

const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A0;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A6;

const unsigned int MOTOR_LEFT_ANKLE_PIN = 3;   
const unsigned int MOTOR_RIGHT_ANKLE_PIN = 10; 

const unsigned int RED = 22;
const unsigned int GREEN = 23;
const unsigned int BLUE = 24;
const unsigned int LED_PIN = 25; //POWER LED

const unsigned int MOTOR_ENABLE_PIN = 5;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 4;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 11;

const unsigned int MOTOR_CURRENT_LEFT_ANKLE_PIN = A7; 
const unsigned int MOTOR_CURRENT_RIGHT_ANKLE_PIN = A1; 

const unsigned int PWR_SCL = A5;   //SCL
const unsigned int PWR_SDA = A4;   //SDA
#endif

#if BOARD_VERSION == TMOTOR_REV1
#define ENABLE_PWM

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A2;
const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A3;

const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A0;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A6;

//All of the _ref variables are used by arduino. They were placed here to indicate their occupancy. 
const uint8_t SCK_ref = 13;
const uint8_t MISO_ref = 12;
const uint8_t MOSI_ref = 11;
const uint8_t CS_ref = 10; //To Change this go to the mcp2515.h file and change CS define
const uint8_t OE = 9;
const uint8_t MCP_INT = 8;

const unsigned int RED = 22;
const unsigned int GREEN = 23;
const unsigned int BLUE = 24;
const unsigned int LED_PIN = 25; //POWER LED


const unsigned int PWR_SCL = A5;   //SCL
const unsigned int PWR_SDA = A4;   //SDA
#endif

#ifdef ENABLE_PWM
const bool PWM_CONTROL = true;
#else
const bool PWM_CONTROL = false;
#endif

#endif
