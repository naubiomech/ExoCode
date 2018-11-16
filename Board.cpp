#include <Arduino.h>
#include "Board.hpp"
#include "Pins.hpp"
#include <i2c_t3.h>

ExoPins* setupPins();

ExoSystem* setupBoard(){
  // enable bluetooth
  Serial.begin(115200);
  Serial.println("Starting");
  // The led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  analogWriteResolution(12);
  analogReadResolution(12);

  // set pin mode for left and right sides
  pinMode(MOTOR_ENABLE_PIN, OUTPUT); //Enable disable the motors
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  ExoPins* exoPins = setupPins();
  ExoSystem* exoSystem = new ExoSystem(exoPins);
  delete exoPins;
  // Fast torque calibration
  exoSystem->exo->calibrateTorque();
  digitalWrite(LED_PIN, LOW);
  Serial.println("Exo Ready");
  return exoSystem;
}

#ifdef QUAD_BOARD
ExoPins* setupQuadBoardPins(){
  int motor_count = 2;
  int fsr_groups_per_leg = 1;
  int fsrs_per_group = 2;
  int imu_count = 0;
  ExoPins* exo_pins = new ExoPins(motor_count, fsr_groups_per_leg, fsrs_per_group, imu_count);

  exo_pins->bluetooth_rx = BLUETOOTH_RX_PIN;
  exo_pins->bluetooth_tx = BLUETOOTH_TX_PIN;

  exo_pins->left_leg->joint_pins[0]->motor_pins->motor = MOTOR_LEFT_ANKLE_PIN;
  exo_pins->left_leg->joint_pins[0]->motor_pins->err = MOTOR_ERROR_LEFT_ANKLE_PIN;
  exo_pins->left_leg->joint_pins[0]->torque_sensor_pins->torque_sensor = TORQUE_SENSOR_LEFT_ANKLE_PIN;
  exo_pins->left_leg->joint_pins[1]->motor_pins->motor = MOTOR_LEFT_KNEE_PIN;
  exo_pins->left_leg->joint_pins[1]->motor_pins->err = MOTOR_ERROR_LEFT_KNEE_PIN;
  exo_pins->left_leg->joint_pins[1]->torque_sensor_pins->torque_sensor = TORQUE_SENSOR_LEFT_KNEE_PIN;
  exo_pins->left_leg->fsr_groups_pins[0]->fsr_pins[0] = FSR_SENSE_LEFT_TOE_PIN;
  exo_pins->left_leg->fsr_groups_pins[0]->fsr_pins[1] = FSR_SENSE_LEFT_HEEL_PIN;

  exo_pins->right_leg->joint_pins[0]->motor_pins->motor = MOTOR_RIGHT_ANKLE_PIN;
  exo_pins->right_leg->joint_pins[0]->motor_pins->err = MOTOR_ERROR_RIGHT_ANKLE_PIN;
  exo_pins->right_leg->joint_pins[0]->torque_sensor_pins->torque_sensor = TORQUE_SENSOR_RIGHT_ANKLE_PIN;
  exo_pins->right_leg->joint_pins[1]->motor_pins->motor = MOTOR_RIGHT_KNEE_PIN;
  exo_pins->right_leg->joint_pins[1]->motor_pins->err = MOTOR_ERROR_RIGHT_KNEE_PIN;
  exo_pins->right_leg->joint_pins[1]->torque_sensor_pins->torque_sensor = TORQUE_SENSOR_RIGHT_KNEE_PIN;
  exo_pins->right_leg->fsr_groups_pins[0]->fsr_pins[0] = FSR_SENSE_RIGHT_TOE_PIN;
  exo_pins->right_leg->fsr_groups_pins[0]->fsr_pins[1] = FSR_SENSE_RIGHT_HEEL_PIN;

  return exo_pins;
}
#endif

#ifdef IMU_BOARD
ExoPins* setupIMUBoardPins(){
  return NULL;
}
#endif

#ifdef TWO_LEG_BOARD
ExoPins* setupTwoLegBoardPins(){
  return NULL;
}
#endif


ExoPins* setupPins(){
#ifdef QUAD_BOARD
  return setupQuadBoardPins();
#endif
#ifdef IMU_BOARD
  return setupIMUBoardPins();
#endif
#ifdef TWO_LEG_BOARD
  return setupTwoLegBoardPins();
#endif
  return NULL;
}

