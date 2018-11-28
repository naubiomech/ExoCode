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

Exoskeleton* QuadExoBuilderDirector::build(Board* board){
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
  ->finishLeg()
  ->build();
  return exo;
};


ExoBuilder* ExoBuilder::addTransceiver(TxPort* tx, RxPort* rx){
  this->rx = rx;
  this->tx = tx;
  return this;
}

Exoskeleton* ExoBuilder::build(){
  Leg* left_leg = left_builder->build();
  Leg* right_leg = right_builder->build();
  Transceiver* transceiver = new MatlabTransceiver(tx, rx);
  Exoskeleton* exo = new Exoskeleton(left_leg, right_leg, transceiver);
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

LegBuilder* LegBuilder::addStateMachine(State* states){
  states = states;
  return this;
}

LegBuilder* LegBuilder::addJoint(InputPort* torque_sensor_port, OutputPort* motor_port,
                                 InputPort* error_port, ControlModule* module){
  torque_sensor_ports.push_back(torque_sensor_port);
  motor_ports.push_back(motor_port);
  error_ports.push_back(error_port);
  controls.push_back(module);
  return this;
}

LegBuilder* LegBuilder::beginFSRGroup(){
  fsr_ports_begin.clear();
  return this;
}

LegBuilder* LegBuilder::finishFSRGroup(){
  fsr_ports.push_back(fsr_ports_begin);
  return this;
}

LegBuilder* LegBuilder::addFSR(InputPort* fsr_port){
  fsr_ports_begin.push_back(fsr_port);
  return this;
}

ExoBuilder* LegBuilder::finishLeg(){
  return return_context;
}

Leg* LegBuilder::build(){
  std::vector<Joint*> joints;
  for (unsigned int i = 0; i < motor_ports.size(); i++){
    Motor* motor = new Motor(error_ports[i], motor_ports[i], sign);
    TorqueSensor* torque_sensor = new TorqueSensor(torque_sensor_ports[i], sign);
    Joint* joint = new Joint(controls[i], motor, torque_sensor);
    joints.push_back(joint);
  }

  std::vector<FSRGroup*> fsrs;
  for (unsigned int i = 0; i < fsr_ports.size(); i++){
    std::vector<FSR*> single_fsrs;
    for (unsigned int j = 0; j < fsr_ports[i].size(); i++){
      single_fsrs.push_back(new FSR(fsr_ports[i][j]));
    }
    FSRGroup* group = new FSRGroup(single_fsrs);
    fsrs.push_back(group);
  }

  std::vector<IMU*> imus;
  for (unsigned int i = 0; i < imu_ports.size(); i++){
    imus.push_back(new IMU(imu_ports[i], imu_address[i]));
  }
  Leg* leg = new Leg(states, joints,fsrs,imus);
  return leg;
}
