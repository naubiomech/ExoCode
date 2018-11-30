#include "ExoBuilder.hpp"
#include "Exoskeleton.hpp"
#include "Leg.hpp"
#include "Joint.hpp"
#include "TorqueSensor.hpp"
#include "Motor.hpp"
#include "Control_Module.hpp"
#include "States.hpp"
#include "Board.hpp"
#include "FSR.hpp"

Exoskeleton* QuadExoDirector::build(Board* board){
  StateBuilder* state_builder = new StateBuilder();
  state_builder
    ->addState(SWING)
    ->addState(LATE_STANCE);


  ControlModuleBuilder* module_builder = new ControlModuleBuilder();
  module_builder
    ->addState(SWING, zero_torque)
    ->addState(LATE_STANCE, bang_bang);
  ExoBuilder* builder = new ExoBuilder();
  Exoskeleton* exo = builder
    ->addTransceiver(board->takeBluetoothTxPort(), board->takeBluetoothRxPort())
    ->addMotorEnable(board->takeMotorEnablePort())
    ->addLedPort(board->takeLedPort())
    ->beginLeftLeg()
    ->addStateMachine(state_builder->build())
    ->addJoint(board->takeTorqueSensorLeftAnklePort(),
               board->takeMotorLeftAnklePort(),
               board->takeMotorErrorLeftAnklePort(),
               module_builder->build(LATE_STANCE))
    ->addJoint(board->takeTorqueSensorLeftKneePort(),
               board->takeMotorLeftKneePort(),
               board->takeMotorErrorLeftKneePort(),
               module_builder->build(LATE_STANCE))
    ->beginFSRGroup()
    ->addFSR(board->takeFsrSenseLeftToePort())
    ->addFSR(board->takeFsrSenseLeftHeelPort())
    ->finishFSRGroup()
    ->finishLeg()
    ->beginRightLeg()
    ->addStateMachine(state_builder->build())
    ->addJoint(board->takeTorqueSensorRightAnklePort(),
               board->takeMotorRightAnklePort(),
               board->takeMotorErrorRightAnklePort(),
               module_builder->build(LATE_STANCE))
    ->addJoint(board->takeTorqueSensorRightKneePort(),
               board->takeMotorRightKneePort(),
               board->takeMotorErrorRightKneePort(),
               module_builder->build(LATE_STANCE))
    ->beginFSRGroup()
    ->addFSR(board->takeFsrSenseRightToePort())
    ->addFSR(board->takeFsrSenseRightHeelPort())
    ->finishFSRGroup()
    ->finishLeg();
  delete state_builder;
  delete module_builder;
  Exoskeleton* exo = builder->build();
  delete builder;
  return exo;
};


ExoBuilder* ExoBuilder::addTransceiver(TxPort* tx, RxPort* rx){
  this->rx = rx;
  this->tx = tx;
  return this;
}

ExoBuilder* ExoBuilder::addMotorEnable(OutputPort* motor_enable_port){
  this->motor_enable_port = motor_enable_port;
  return this;
}

ExoBuilder* ExoBuilder::addLedPort(OutputPort* led_port){
  this->led_port = led_port;
  return this;
}

ExoBuilder::~ExoBuilder(){
  delete left_builder;
  delete right_builder;
}

Exoskeleton* ExoBuilder::build(){
  Leg* left_leg = left_builder->build();
  delete left_builder;
  left_builder = NULL;
  Leg* right_leg = right_builder->build();
  delete right_builder;
  right_builder = NULL;
  Transceiver* transceiver = new MatlabTransceiver(tx, rx);
  Exoskeleton* exo = new Exoskeleton(left_leg, right_leg, transceiver, motor_enable_port, led_port);
  return exo;
}

LegBuilder* ExoBuilder::beginRightLeg(){
  right_builder = new LegBuilder(this);
  return right_builder;
}

LegBuilder* ExoBuilder::beginLeftLeg(){
  left_builder = new LegBuilder(this);
  return left_builder;
}

LegBuilder::LegBuilder(ExoBuilder* return_context){
  this->return_context = return_context;
}

LegBuilder::~LegBuilder(){
}

LegBuilder* LegBuilder::addStateMachine(State* states){
  states = states;
  return this;
}

LegBuilder* LegBuilder::addJoint(InputPort* torque_sensor_port, OutputPort* motor_port,
                                 InputPort* error_port, ControlModule* module){
  torque_sensor_ports.append(torque_sensor_port);
  motor_ports.append(motor_port);
  error_ports.append(error_port);
  controls.append(module);
  return this;
}

LegBuilder* LegBuilder::beginFSRGroup(){
  fsr_ports_begin.clear();
  return this;
}

  LegBuilder* LegBuilder::finishFSRGroup(){
    fsr_ports.append(fsr_ports_begin);
    return this;
  }

  LegBuilder* LegBuilder::addFSR(InputPort* fsr_port){
    fsr_ports_begin.append(fsr_port);
    return this;
  }

  ExoBuilder* LegBuilder::finishLeg(){
    return return_context;
  }

  Leg* LegBuilder::build(){
    LinkedList<Joint*> joints;
    for (unsigned int i = 0; i < motor_ports.size(); i++){
      Motor* motor = new Motor(error_ports[i], motor_ports[i], sign);
      TorqueSensor* torque_sensor = new TorqueSensor(torque_sensor_ports[i], sign);
      Joint* joint = new Joint(controls[i], motor, torque_sensor);
      joints.append(joint);
    }

    LinkedList<FSRGroup*> fsrs;
    for (unsigned int i = 0; i < fsr_ports.size(); i++){
      LinkedList<FSR*> single_fsrs;
      for (unsigned int j = 0; j < fsr_ports[i].size(); i++){
        single_fsrs.append(new FSR(fsr_ports[i][j]));
      }
      FSRGroup* group = new FSRGroup(single_fsrs);
      fsrs.append(group);
    }

    LinkedList<IMU*> imus;
    for (unsigned int i = 0; i < imu_ports.size(); i++){
      imus.append(new IMU(imu_ports[i], imu_address[i]));
    }
    Leg* leg = new Leg(states, joints,fsrs,imus);
    return leg;
  }
