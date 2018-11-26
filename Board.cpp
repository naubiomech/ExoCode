#include <Arduino.h>
#include "Board.hpp"
#include "Ports.hpp"
#include <i2c_t3.h>

ExoPins* setupPins();

class BoardDirector{
public:
  virtual Board* build() = 0;
}

class QuadBoardDirector:public BoardDirector{
public:
  Board* build();
}

Board* QuadBoardDirector::build(){
  BoardBuilder* builder = new BoardBuilder(new ArduinoPortFactory());
  Board* board = builder
    .setAnalogWriteResolution(10)
    .setAnalogReadResolution(10)
    .setBluetoothTxPort(0)
    .setBluetoothRxPort(1)
    .setLedPort(13)
    .setFsrSenseRightHeelPort(A12)
    .setFsrSenseRightToePort(A13)
    .setFsrSenseLeftHeelPort(A14)
    .setFsrSenseLeftToePort(A15)
    .setTorqueSensorRightAnklePort(A0)
    .setTorqueSensorRightKneePort(A1)
    .setTorqueSensorLeftAnklePort(A6)
    .setTorqueSensorLeftKneePort(A5)
    .setMotorLeftKneePort(23)
    .setMotorLeftAnklePort(22)
    .setMotorRightKneePort(5)
    .setMotorRightAnklePort(6)
    .setMotorEnablePort(16)
    .setMotorErrorLeftKneePin(24)
    .setMotorErrorLeftAnklePort(25)
    .setMotorErrorRightKneePin(26)
    .setMotorErrorRightAnklePort(27)
    .build();
}

class BoardBuilder{
public:
  BoardBuilder(PortFactory* factory);
  Board* build();
  void reset();
  void setBluetoothTxPort(unsigned int pin);
  void setBluetoothRxPort(unsigned int pin);
  void setFsrSenseLeftToePort(unsigned int pin);
  void setFsrSenseLeftHeelPort(unsigned int pin);
  void setFsrSenseRightToePort(unsigned int pin);
  void setFsrSenseRightHeelPort(unsigned int pin);
  void setTorqueSensorLeftKneePort(unsigned int pin);
  void setTorqueSensorLeftAnklePort(unsigned int pin);
  void setTorqueSensorRightKneePort(unsigned int pin);
  void setTorqueSensorRightAnklePort(unsigned int pin);
  void setMotorLeftKneePort(unsigned int pin);
  void setMotorLeftAnklePort(unsigned int pin);
  void setMotorRightKneePort(unsigned int pin);
  void setMotorRightAnklePort(unsigned int pin);
  void setLedPort(unsigned int pin);
  void setMotorEnablePort(unsigned int pin);
  void setMotorErrorLeftKneePort(unsigned int pin);
  void setMotorErrorLeftAnklePort(unsigned int pin);
  void setMotorErrorRightKneePort(unsigned int pin);
  void setMotorErrorRightAnklePort(unsigned int pin);
};

BoardBuilder(PortFactory* factory){
  port_factory = factory;
  read_resolution = 8;
  write_resolution = 8;
}

Board* BoardBuilder::build(){
  return board;
}

BoardBuilder* BoardBuilder::reset(){
  board = new Board();
  return this;
}

BoardBuilder* BoardBuilder::setAnalogWriteResolution(unsigned int bits){
  write_resolution = bits;
  analogWriteResolution(bits);
}

BoardBuilder* BoardBuilder::setAnalogReadResolution(unsigned int bits){
  read_resolution = bits;
  analogReadResolution(bits);
}

BoardBuilder* BoardBuilder::setBluetoothTxPort(unsigned int pin){
  board->setBluetoothTxPort(port_factory->createDigitalOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setBluetoothRxPort(unsigned int pin){
  board->setBluetoothRxPort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseLeftToePort(unsigned int pin){
  board->setFsrSenseLeftToePort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseLeftHeelPort(unsigned int pin){
  board->setFsrSenseLeftHeelPort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseRightToePort(unsigned int pin){
  board->setFsrSenseRightToePort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseRightHeelPort(unsigned int pin){
  board->setFsrSenseRightHeelPort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorLeftKneePort(unsigned int pin){
  board->setTorqueSensorLeftKneePort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorLeftAnklePort(unsigned int pin){
  board->setTorqueSensorLeftAnklePort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorRightKneePort(unsigned int pin){
  board->setTorqueSensorRightKneePort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorRightAnklePort(unsigned int pin){
  board->setTorqueSensorRightAnklePort(port_factory->createAnalogInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorLeftKneePort(unsigned int pin){
  board->setMotorLeftKneePort(port_factory->createPwmOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorLeftAnklePort(unsigned int pin){
  board->setMotorLeftAnklePort(port_factory->createPwmOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorRightKneePort(unsigned int pin){
  board->setMotorRightKneePort(port_factory->createPwmOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorRightAnklePort(unsigned int pin){
  board->setMotorRightAnklePort(port_factory->createPwmOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setLedPort(unsigned int pin){
  board->setLedPort(port_factory->createDigitalOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorEnablePort(unsigned int pin){
  board->setMotorEnablePort(port_factory->createDigitalOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorLeftKneePort(unsigned int pin){
  board->setMotorErrorLeftKneePort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorLeftAnklePort(unsigned int pin){
  board->setMotorErrorLeftAnklePort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorRightKneePort(unsigned int pin){
  board->setMotorErrorRightKneePort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorRightAnklePort(unsigned int pin){
  board->setMotorErrorRightAnklePort(port_factory->createDigitalInputPort(pin));
  return this;
}

Board::Board(){
  bluetooth_tx_port = NULL;
  bluetooth_rx_port = NULL;
  fsr_sense_left_toe_port = NULL;
  fsr_sense_left_heel_port = NULL;
  fsr_sense_right_toe_port = NULL;
  fsr_sense_right_heel_port = NULL;
  torque_sensor_left_knee_port = NULL;
  torque_sensor_left_ankle_port = NULL;
  torque_sensor_right_knee_port = NULL;
  torque_sensor_right_ankle_port = NULL;
  motor_left_knee_port = NULL;
  motor_left_ankle_port = NULL;
  motor_right_knee_port = NULL;
  motor_right_ankle_port = NULL;
  led_port = NULL;
  motor_enable_port = NULL;
  motor_error_left_knee_port = NULL;
  motor_error_left_ankle_port = NULL;
  motor_error_right_knee_port = NULL;
  motor_error_right_ankle_port = NULL;
}

Exoskeleton* setupBoard(){
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
  Exoskeleton* exo = new Exoskeleton(exoPins);
  delete exoPins;
  // Fast torque calibration
  exo->calibrateTorque();
  digitalWrite(LED_PIN, LOW);
  Serial.println("Exo Ready");
  return exo;
}

OutputPort* Board::takeBluetoothTxPort(){
  OutputPort* port = bluetooth_tx_port;
  bluetooth_tx_port = NULL;
  return port;
}

InputPort* Board::takeBluetoothRxPort(){
  InputPort* port = bluetooth_rx_port;
  bluetooth_rx_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseLeftToePort(){
  InputPort* port = fsr_sense_left_toe_port;
  fsr_sense_left_toe_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseLeftHeelPort(){
  InputPort* port = fsr_sense_left_heel_port;
  fsr_sense_left_heel_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseRightToePort(){
  InputPort* port = fsr_sense_right_toe_port;
  fsr_sense_right_toe_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseRightHeelPort(){
  InputPort* port = fsr_sense_right_heel_port;
  fsr_sense_right_heel_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorLeftKneePort(){
  InputPort* port = torque_sensor_left_knee_port;
  torque_sensor_left_knee_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorLeftAnklePort(){
  InputPort* port = torque_sensor_left_ankle_port;
  torque_sensor_left_ankle_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorRightKneePort(){
  InputPort* port = torque_sensor_right_knee_port;
  torque_sensor_right_knee_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorRightAnklePort(){
  InputPort* port = torque_sensor_right_ankle_port;
  torque_sensor_right_ankle_port = NULL;
  return port;
}

OutputPort* Board::takeMotorLeftKneePort(){
  OutputPort* port = motor_left_knee_port;
  motor_left_knee_port = NULL;
  return port;
}

OutputPort* Board::takeMotorLeftAnklePort(){
  OutputPort* port = motor_left_ankle_port;
  motor_left_ankle_port = NULL;
  return port;
}

OutputPort* Board::takeMotorRightKneePort(){
  OutputPort* port = motor_right_knee_port;
  motor_right_knee_port = NULL;
  return port;
}

OutputPort* Board::takeMotorRightAnklePort(){
  OutputPort* port = motor_right_ankle_port;
  motor_right_ankle_port = NULL;
  return port;
}

OutputPort* Board::takeLedPort(){
  OutputPort* port = led_port;
  led_port = NULL;
  return port;
}

OutputPort* Board::takeMotorEnablePort(){
  OutputPort* port = motor_enable_port;
  motor_enable_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorLeftKneePort(){
  InputPort* port = motor_error_left_knee_port;
  motor_error_left_knee_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorLeftAnklePort(){
  InputPort* port = motor_error_left_ankle_port;
  motor_error_left_ankle_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorRightKneePort(){
  InputPort* port = motor_error_right_knee_port;
  motor_error_right_knee_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorRightAnklePort(){
  InputPort* port = motor_error_right_ankle_port;
  motor_error_right_ankle_port = NULL;
  return port;
}

void setBluetoothTxPort(OutputPort* port){
  bluetooth_tx_port = port;
}

void setBluetoothRxPort(InputPort* port){
  bluetooth_rx_port = port;
}

void setFsrSenseLeftToePort(InputPort* port){
  fsr_sense_left_toe_port = port;
}

void setFsrSenseLeftHeelPort(InputPort* port){
  fsr_sense_left_heel_port = port;
}

void setFsrSenseRightToePort(InputPort* port){
  fsr_sense_right_toe_port = port;
}

void setFsrSenseRightHeelPort(InputPort* port){
  fsr_sense_right_heel_port = port;
}

void setTorqueSensorLeftKneePort(InputPort* port){
  torque_sensor_left_knee_port = port;
}

void setTorqueSensorLeftAnklePort(InputPort* port){
  torque_sensor_left_ankle_port = port;
}

void setTorqueSensorRightKneePort(InputPort* port){
  torque_sensor_right_knee_port = port;
}

void setTorqueSensorRightAnklePort(InputPort* port){
  torque_sensor_right_ankle_port = port;
}

void setMotorLeftKneePort(OutputPort* port){
  motor_left_knee_port = port;
}

void setMotorLeftAnklePort(OutputPort* port){
  motor_left_ankle_port = port;
}

void setMotorRightKneePort(OutputPort* port){
  motor_right_knee_port = port;
}

void setMotorRightAnklePort(OutputPort* port){
  motor_right_ankle_port = port;
}

void setLedPort(OutputPort* port){
  led_port = port;
}

void setMotorEnablePort(OutputPort* port){
  motor_enable_port = port;
}

void setMotorErrorLeftKneePort(InputPort* port){
  motor_error_left_knee_port = port;
}

void setMotorErrorLeftAnklePort(InputPort* port){
  motor_error_left_ankle_port = port;
}

void setMotorErrorRightKneePort(InputPort* port){
  motor_error_right_knee_port = port;
}

void setMotorErrorRightAnklePort(InputPort* port){
  motor_error_right_ankle_port = port;
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

