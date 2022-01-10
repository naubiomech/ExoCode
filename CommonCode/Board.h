#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER

#if BOARD_VERSION == AK_Board_V0_1
#include "TSPISlave.h"

  // Serial Pins, NC
  const unsigned int RX1_PIN = 0;
  const unsigned int TX1_PIN = 1;
  
  // CAN Pins
  const unsigned int CAN_RX_PIN = 4;
  const unsigned int CAN_TX_PIN = 3;
  
  // FSR Pins
  const unsigned int FSR_SENSE_RIGHT_HEEL_PIN = A7;
  const unsigned int FSR_SENSE_RIGHT_TOE_PIN = A6;
  const unsigned int FSR_SENSE_LEFT_TOE_PIN = A15;
  const unsigned int FSR_SENSE_LEFT_HEEL_PIN = A14;
  
  // Torque Sensor Pins
  const unsigned int TORQUE_SENSOR_RIGHT = A9;
  const unsigned int TORQUE_SENSOR_RIGHT1 = A8;
  const unsigned int TORQUE_SENSOR_LEFT = A17;
  const unsigned int TORQUE_SENSOR_LEFT = A16;
  
  // Sync LED Pins
  const unsigned int SYNC_LED_PIN = 29;
  const unsigned int SYNC_DEFAULT_PIN = 25;
  
  // Status LED Pins
  const unsigned int STATUS_LED_R_PIN = 28;
  const unsigned int STATUS_LED_G_PIN = 27;
  const unsigned int STATUS_LED_B_PIN = 26;
  
  // SPI Follower Pins
  const unsigned int MISO_PIN = 12;
  const unsigned int MOSI_PIN = 11;
  const unsigned int SCK_PIN = 13;
  const unsigned int CS_PIN = 10;
  const unsigned int SPI_MODE = 16;
  
  // Pin to Stop the Motors
  const unsigned int MOTOR_STOP_PIN = 6;

#endif

#endif
