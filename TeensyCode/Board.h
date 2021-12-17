#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER

#ifdef IMU_BOARD
#include <i2c_t3.h>
const i2c_pins IMU_SLOT_1_PINS = I2C_PINS_3_4;
const i2c_pins IMU_SLOT_2_PINS = I2C_PINS_7_8;
const i2c_pins IMU_SLOT_3_PINS = I2C_PINS_37_38;

const unsigned int RX1_PIN = 0;
const unsigned int TX1_PIN = 1;

const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A12;
const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A13;
const unsigned int FSR_SENSE_LEFT_TOE_PIN = A15;
const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A14;

const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A6;
const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A0;

const unsigned int MOTOR_LEFT_ANKLE_PIN = A21;
const unsigned int MOTOR_RIGHT_ANKLE_PIN = A22;

const unsigned int LED_PIN = 13;

const unsigned int MOTOR_ENABLE_PIN = A3;

const unsigned int WHICH_LEG_PIN = 15;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 20;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 21;
#endif

#ifdef TWO_LEG_BOARD
const unsigned int RX1_PIN = 0;
const unsigned int TX1_PIN = 1;

const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A12;
const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A13;
const unsigned int FSR_SENSE_LEFT_TOE_PIN = A15;
const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A14;

const unsigned int TORQUE_SENSOR_RIGHT_ANKLE_PIN = A18;
const unsigned int TORQUE_SENSOR_LEFT_ANKLE_PIN = A19;

const unsigned int MOTOR_LEFT_ANKLE_PIN = A21;
const unsigned int MOTOR_RIGHT_ANKLE_PIN = A22;

const unsigned int LED_PIN = 13;

const unsigned int MOTOR_ENABLE_PIN = 22;

const unsigned int WHICH_LEG_PIN = 15;

const unsigned int MOTOR_ERROR_LEFT_ANKLE_PIN = 20;
const unsigned int MOTOR_ERROR_RIGHT_ANKLE_PIN = 21;
#endif

#ifdef QUAD_BOARD
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

#ifdef DUAL_BOARD
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

const unsigned int BREAKOUT_LEFT_PIN = A10;
const unsigned int BREAKOUT_RIGHT_PIN = A11;
#endif

#ifdef ENABLE_PWM
const bool PWM_CONTROL = true;
#else
const bool PWM_CONTROL = false;
#endif



#endif
