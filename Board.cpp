#include <Arduino.h>
#include "Board.hpp"
#include "Port.hpp"
#include <i2c_t3.h>
#include "Linked_List.hpp"

class BoardBuilder{
private:
  Board* board;
  PortFactory* port_factory;
  unsigned int read_resolution;
  unsigned int write_resolution;
public:
  BoardBuilder(PortFactory* factory);
  Board* build();
  BoardBuilder* reset();
  BoardBuilder* setAnalogReadResolution(unsigned int bits);
  BoardBuilder* setAnalogWriteResolution(unsigned int bits);
  BoardBuilder* setBluetoothTxPort(unsigned int pin);
  BoardBuilder* setBluetoothRxPort(unsigned int pin);
  BoardBuilder* setFsrSenseLeftToePort(unsigned int pin);
  BoardBuilder* setFsrSenseLeftHeelPort(unsigned int pin);
  BoardBuilder* setFsrSenseRightToePort(unsigned int pin);
  BoardBuilder* setFsrSenseRightHeelPort(unsigned int pin);
  BoardBuilder* setTorqueSensorLeftKneePort(unsigned int pin);
  BoardBuilder* setTorqueSensorLeftAnklePort(unsigned int pin);
  BoardBuilder* setTorqueSensorRightKneePort(unsigned int pin);
  BoardBuilder* setTorqueSensorRightAnklePort(unsigned int pin);
  BoardBuilder* setMotorLeftKneePort(unsigned int pin);
  BoardBuilder* setMotorLeftAnklePort(unsigned int pin);
  BoardBuilder* setMotorRightKneePort(unsigned int pin);
  BoardBuilder* setMotorRightAnklePort(unsigned int pin);
  BoardBuilder* setLedPort(unsigned int pin);
  BoardBuilder* setMotorEnablePort(unsigned int pin);
  BoardBuilder* setMotorErrorLeftKneePort(unsigned int pin);
  BoardBuilder* setMotorErrorLeftAnklePort(unsigned int pin);
  BoardBuilder* setMotorErrorRightKneePort(unsigned int pin);
  BoardBuilder* setMotorErrorRightAnklePort(unsigned int pin);
};

Board* QuadBoardDirector::build(){
  BoardBuilder* builder = new BoardBuilder(new ArduinoPortFactory());
  Board* board = builder
    ->setAnalogWriteResolution(10)
    ->setAnalogReadResolution(10)
    ->setBluetoothTxPort(0)
    ->setBluetoothRxPort(1)
    ->setLedPort(13)
    ->setFsrSenseRightHeelPort(A12)
    ->setFsrSenseRightToePort(A13)
    ->setFsrSenseLeftHeelPort(A14)
    ->setFsrSenseLeftToePort(A15)
    ->setTorqueSensorRightAnklePort(A0)
    ->setTorqueSensorRightKneePort(A1)
    ->setTorqueSensorLeftAnklePort(A6)
    ->setTorqueSensorLeftKneePort(A5)
    ->setMotorLeftKneePort(23)
    ->setMotorLeftAnklePort(22)
    ->setMotorRightKneePort(5)
    ->setMotorRightAnklePort(6)
    ->setMotorEnablePort(16)
    ->setMotorErrorLeftKneePort(24)
    ->setMotorErrorLeftAnklePort(25)
    ->setMotorErrorRightKneePort(26)
    ->setMotorErrorRightAnklePort(27)
    ->build();
  return board;
}

BoardBuilder::BoardBuilder(PortFactory* factory){
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
  return this;
}

BoardBuilder* BoardBuilder::setAnalogReadResolution(unsigned int bits){
  read_resolution = bits;
  analogReadResolution(bits);
  return this;
}

BoardBuilder* BoardBuilder::setBluetoothTxPort(unsigned int pin){
  board->setBluetoothTxPort(port_factory->createTxPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setBluetoothRxPort(unsigned int pin){
  board->setBluetoothRxPort(port_factory->createRxPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseLeftToePort(unsigned int pin){
  board->setFsrSenseLeftToePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseLeftHeelPort(unsigned int pin){
  board->setFsrSenseLeftHeelPort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseRightToePort(unsigned int pin){
  board->setFsrSenseRightToePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseRightHeelPort(unsigned int pin){
  board->setFsrSenseRightHeelPort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorLeftKneePort(unsigned int pin){
  board->setTorqueSensorLeftKneePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorLeftAnklePort(unsigned int pin){
  board->setTorqueSensorLeftAnklePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorRightKneePort(unsigned int pin){
  board->setTorqueSensorRightKneePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorRightAnklePort(unsigned int pin){
  board->setTorqueSensorRightAnklePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorLeftKneePort(unsigned int pin){
  board->setMotorLeftKneePort(port_factory->createPwmOutputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorLeftAnklePort(unsigned int pin){
  board->setMotorLeftAnklePort(port_factory->createPwmOutputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorRightKneePort(unsigned int pin){
  board->setMotorRightKneePort(port_factory->createPwmOutputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorRightAnklePort(unsigned int pin){
  board->setMotorRightAnklePort(port_factory->createPwmOutputPort(pin, write_resolution));
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

TxPort* Board::takeBluetoothTxPort(){
  TxPort* port = bluetooth_tx_port;
  bluetooth_tx_port = NULL;
  return port;
}

RxPort* Board::takeBluetoothRxPort(){
  RxPort* port = bluetooth_rx_port;
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

void Board::setBluetoothTxPort(TxPort* port){
  bluetooth_tx_port = port;
}

void Board::setBluetoothRxPort(RxPort* port){
  bluetooth_rx_port = port;
}

void Board::setFsrSenseLeftToePort(InputPort* port){
  fsr_sense_left_toe_port = port;
}

void Board::setFsrSenseLeftHeelPort(InputPort* port){
  fsr_sense_left_heel_port = port;
}

void Board::setFsrSenseRightToePort(InputPort* port){
  fsr_sense_right_toe_port = port;
}

void Board::setFsrSenseRightHeelPort(InputPort* port){
  fsr_sense_right_heel_port = port;
}

void Board::setTorqueSensorLeftKneePort(InputPort* port){
  torque_sensor_left_knee_port = port;
}

void Board::setTorqueSensorLeftAnklePort(InputPort* port){
  torque_sensor_left_ankle_port = port;
}

void Board::setTorqueSensorRightKneePort(InputPort* port){
  torque_sensor_right_knee_port = port;
}

void Board::setTorqueSensorRightAnklePort(InputPort* port){
  torque_sensor_right_ankle_port = port;
}

void Board::setMotorLeftKneePort(OutputPort* port){
  motor_left_knee_port = port;
}

void Board::setMotorLeftAnklePort(OutputPort* port){
  motor_left_ankle_port = port;
}

void Board::setMotorRightKneePort(OutputPort* port){
  motor_right_knee_port = port;
}

void Board::setMotorRightAnklePort(OutputPort* port){
  motor_right_ankle_port = port;
}

void Board::setLedPort(OutputPort* port){
  led_port = port;
}

void Board::setMotorEnablePort(OutputPort* port){
  motor_enable_port = port;
}

void Board::setMotorErrorLeftKneePort(InputPort* port){
  motor_error_left_knee_port = port;
}

void Board::setMotorErrorLeftAnklePort(InputPort* port){
  motor_error_left_ankle_port = port;
}

void Board::setMotorErrorRightKneePort(InputPort* port){
  motor_error_right_knee_port = port;
}

void Board::setMotorErrorRightAnklePort(InputPort* port){
  motor_error_right_ankle_port = port;
}

Board::~Board(){
  delete portFactory;
  delete bluetooth_tx_port;
  delete bluetooth_rx_port;
  delete fsr_sense_left_toe_port;
  delete fsr_sense_left_heel_port;
  delete fsr_sense_right_toe_port;
  delete fsr_sense_right_heel_port;
  delete torque_sensor_left_knee_port;
  delete torque_sensor_left_ankle_port;
  delete torque_sensor_right_knee_port;
  delete torque_sensor_right_ankle_port;
  delete motor_left_knee_port;
  delete motor_left_ankle_port;
  delete motor_right_knee_port;
  delete motor_right_ankle_port;
  delete led_port;
  delete motor_enable_port;
  delete motor_error_left_knee_port;
  delete motor_error_left_ankle_port;
  delete motor_error_right_knee_port;
  delete motor_error_right_ankle_port;
}
