#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER
#include "Parameters.hpp"
#include "Exoskeleton.hpp"

class Board{
private:
  PortFactory* portFactory;
  TxPort* bluetooth_tx_port;
  RxPort* bluetooth_rx_port;
  InputPort* fsr_sense_left_toe_port;
  InputPort* fsr_sense_left_heel_port;
  InputPort* fsr_sense_right_toe_port;
  InputPort* fsr_sense_right_heel_port;
  InputPort* torque_sensor_left_knee_port;
  InputPort* torque_sensor_left_ankle_port;
  InputPort* torque_sensor_right_knee_port;
  InputPort* torque_sensor_right_ankle_port;
  OutputPort* motor_left_knee_port;
  OutputPort* motor_left_ankle_port;
  OutputPort* motor_right_knee_port;
  OutputPort* motor_right_ankle_port;
  OutputPort* led_port;
  OutputPort* motor_enable_port;
  InputPort* motor_error_left_knee_port;
  InputPort* motor_error_left_ankle_port;
  InputPort* motor_error_right_knee_port;
  InputPort* motor_error_right_ankle_port;

public:
  Board();

  TxPort* takeBluetoothTxPort();
  RxPort* takeBluetoothRxPort();
  InputPort* takeFsrSenseLeftToePort();
  InputPort* takeFsrSenseLeftHeelPort();
  InputPort* takeFsrSenseRightToePort();
  InputPort* takeFsrSenseRightHeelPort();
  InputPort* takeTorqueSensorLeftKneePort();
  InputPort* takeTorqueSensorLeftAnklePort();
  InputPort* takeTorqueSensorRightKneePort();
  InputPort* takeTorqueSensorRightAnklePort();
  OutputPort* takeMotorLeftKneePort();
  OutputPort* takeMotorLeftAnklePort();
  OutputPort* takeMotorRightKneePort();
  OutputPort* takeMotorRightAnklePort();
  OutputPort* takeLedPort();
  OutputPort* takeMotorEnablePort();
  InputPort* takeMotorErrorLeftKneePort();
  InputPort* takeMotorErrorLeftAnklePort();
  InputPort* takeMotorErrorRightKneePort();
  InputPort* takeMotorErrorRightAnklePort();

  void setBluetoothTxPort(TxPort* port);
  void setBluetoothRxPort(RxPort* port);
  void setFsrSenseLeftToePort(InputPort* port);
  void setFsrSenseLeftHeelPort(InputPort* port);
  void setFsrSenseRightToePort(InputPort* port);
  void setFsrSenseRightHeelPort(InputPort* port);
  void setTorqueSensorLeftKneePort(InputPort* port);
  void setTorqueSensorLeftAnklePort(InputPort* port);
  void setTorqueSensorRightKneePort(InputPort* port);
  void setTorqueSensorRightAnklePort(InputPort* port);
  void setMotorLeftKneePort(OutputPort* port);
  void setMotorLeftAnklePort(OutputPort* port);
  void setMotorRightKneePort(OutputPort* port);
  void setMotorRightAnklePort(OutputPort* port);
  void setLedPort(OutputPort* port);
  void setMotorEnablePort(OutputPort* port);
  void setMotorErrorLeftKneePort(InputPort* port);
  void setMotorErrorLeftAnklePort(InputPort* port);
  void setMotorErrorRightKneePort(InputPort* port);
  void setMotorErrorRightAnklePort(InputPort* port);
};

Exoskeleton* setupBoard();

class BoardDirector{
public:
  virtual Board* build() = 0;
};

class QuadBoardDirector:public BoardDirector{
public:
  Board* build();
};

#endif
