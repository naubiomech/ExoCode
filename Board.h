#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER

#define DUAL_BOARD 4
#define DUAL_BOARD_REV3 3
#define QUAD_BOARD 2
#define DUAL_BOARD_REV4 1
#define DUAL_BOARD_REV4_1 5

#if BOARD_VERSION == DUAL_BOARD_REV3
#define ENABLE_PWM
const unsigned int BLUETOOTH_TX_PIN = 34;
const unsigned int BLUETOOTH_RX_PIN = 35;

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A12;
const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A13;

const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A10;
const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A11;

const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A8;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A17;

const unsigned int MOTOR_LEFT_ANKLE_PIN = 1; //PWM
const unsigned int MOTOR_RIGHT_ANKLE_PIN = 37; //PWM

const unsigned int LED_PIN = 13;

const unsigned int MOTOR_ENABLE_PIN = 0;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 2;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 14;

const unsigned int MOTOR_CURRENT_LEFT_ANKLE_PIN = A7;
const unsigned int MOTOR_CURRENT_RIGHT_ANKLE_PIN = A16;

const unsigned int MOTOR_SPEED_LEFT_PIN = A6;
const unsigned int MOTOR_SPEED_RIGHT_PIN = A15;

const unsigned int HALL_LEFT_PIN = A9;
const unsigned int HALL_RIGHT_PIN = A14; 

const unsigned int PWR_ADR_0 = 18; //Address 0
const unsigned int PWR_ADR_1 = 19; //Address 1
const unsigned int PWR_SCL = 16;   //SCL 1
const unsigned int PWR_SDA = 17;   //SDA 1
#endif

#if BOARD_VERSION == DUAL_BOARD_REV4
#define ENABLE_PWM
const unsigned int BLUETOOTH_TX_PIN = 31;
const unsigned int BLUETOOTH_RX_PIN = 32;

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A15;
const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A14;

const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A17;
const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A16;

const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A22;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A21;

const unsigned int MOTOR_LEFT_ANKLE_PIN = 2; //PWM
const unsigned int MOTOR_RIGHT_ANKLE_PIN = 10; //PWM

const unsigned int LED_PIN = 13;

const unsigned int MOTOR_ENABLE_PIN = 0;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 1;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 14;

const unsigned int MOTOR_CURRENT_LEFT_ANKLE_PIN = A7;
const unsigned int MOTOR_CURRENT_RIGHT_ANKLE_PIN = A20;

const unsigned int MOTOR_SPEED_LEFT_PIN = A6;
const unsigned int MOTOR_SPEED_RIGHT_PIN = A19;

const unsigned int HALL_LEFT_PIN = A9;
const unsigned int HALL_RIGHT_PIN = A18; 

const unsigned int TRIGGER_PIN = 23;

const unsigned int PWR_ADR_0 = 5; //Address 0
const unsigned int PWR_ADR_1 = 6; //Address 1
const unsigned int PWR_SCL = 19;   //SCL
const unsigned int PWR_SDA = 18;   //SDA


#endif

#if BOARD_VERSION == DUAL_BOARD_REV4_1
#define ENABLE_PWM
const unsigned int BLUETOOTH_TX_PIN = 31;
const unsigned int BLUETOOTH_RX_PIN = 32;

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A15;
const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A14;

const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A17;
const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A16;

const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A22;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A21;

const unsigned int MOTOR_LEFT_ANKLE_PIN = 2; //PWM
const unsigned int MOTOR_RIGHT_ANKLE_PIN = 10; //PWM

const unsigned int LED_PIN = 13;

const unsigned int MOTOR_ENABLE_PIN = 0;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 1;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 14;

const unsigned int MOTOR_CURRENT_LEFT_ANKLE_PIN = A7;
const unsigned int MOTOR_CURRENT_RIGHT_ANKLE_PIN = A20;

const unsigned int MOTOR_SPEED_LEFT_PIN = A6;
const unsigned int MOTOR_SPEED_RIGHT_PIN = A19;

const unsigned int HALL_LEFT_PIN = A9;
const unsigned int HALL_RIGHT_PIN = A18; 

const unsigned int TRIGGER_PIN = 23;

const unsigned int PWR_ADR_0 = 5; //Address 0
const unsigned int PWR_ADR_1 = 6; //Address 1
const unsigned int PWR_SCL = 19;   //SCL
const unsigned int PWR_SDA = 18;   //SDA

const unsigned int SYNC_LED_PIN = 29;

#endif

#if BOARD_VERSION == QUAD_BOARD
#include <i2c_t3.h>
#define ENABLE_PWM
const i2c_pins IMU_SLOT_0_PINS = I2C_PINS_7_8;
const i2c_pins IMU_SLOT_1_PINS = I2C_PINS_37_38;
const i2c_pins IMU_SLOT_2_PINS = I2C_PINS_3_4;
const int IMU_ADDRESS_0 = 0x28;
const int IMU_ADDRESS_1 = 0x29;

const unsigned int BLUETOOTH_TX_PIN = 0;
const unsigned int BLUETOOTH_RX_PIN = 1;

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A12;
const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A13;
const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A14;
const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A15;

const unsigned int TORQUE_SENSOR_LEFT_KNEE_PIN = A5;
const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A6;
const unsigned int TORQUE_SENSOR_RIGHT_KNEE_PIN = A1;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A0;

const unsigned int MOTOR_LEFT_KNEE_PIN = 23; //PWM
const unsigned int MOTOR_LEFT_ANKLE_PIN = 22; //PWM
const unsigned int MOTOR_RIGHT_KNEE_PIN = 5; //PWM
const unsigned int MOTOR_RIGHT_ANKLE_PIN = 6; //PWM

const unsigned int LED_PIN = 13;

const unsigned int MOTOR_ENABLE_PIN = 17;

const unsigned int MOTOR_ERROR_LEFT_KNEE_PIN = 24;
const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 25;
const unsigned int MOTOR_ERROR_RIGHT_KNEE_PIN = 26;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 27;
//const int LEFT_LEG_SIGN = 1;
//const int RIGHT_LEG_SIGN = -1;
#endif

#if BOARD_VERSION == DUAL_BOARD
#define ENABLE_PWM
const unsigned int BLUETOOTH_TX_PIN = 0;
const unsigned int BLUETOOTH_RX_PIN = 1;

const unsigned int FSR_SENSE_LEFT_TOE_PIN = A12;
const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A13;
const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A14;
const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A15;

const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A0;
const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A6;

const unsigned int MOTOR_LEFT_ANKLE_PIN = 22; //PWM
const unsigned int MOTOR_RIGHT_ANKLE_PIN = 6; //PWM

const unsigned int LED_PIN = 13;

const unsigned int MOTOR_ENABLE_PIN = 17;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 25;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 27;

const unsigned int MOTOR_CURRENT_LEFT_ANKLE_PIN = A7;
const unsigned int MOTOR_CURRENT_RIGHT_ANKLE_PIN = A4;

const unsigned int MOTOR_SPEED_LEFT_PIN = A1;
const unsigned int MOTOR_SPEED_RIGHT_PIN = A2;

const unsigned int LEFT_ANKLE_ANGLE_PIN = A16;
const unsigned int RIGHT_ANKLE_ANGLE_PIN = A17;

const unsigned int BREAKOUT_LEFT_PIN = A10;
const unsigned int BREAKOUT_RIGHT_PIN = A11;
#endif

#ifdef ENABLE_PWM
const bool PWM_CONTROL = true;
#else
const bool PWM_CONTROL = false;
#endif



#endif
