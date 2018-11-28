#ifndef EXO_BUILDER_HEADER
#define EXO_BUILDER_HEADER
#include "Port.hpp"
#include "Exoskeleton.hpp"

class LegBuilder;

class ExoBuilder{
private:
  RxPort* rx;
  TxPort* tx;
  LegBuilder* right_builder;
  LegBuilder* left_builder;
public:
  ExoBuilder* addTransceiver(TxPort* tx, RxPort* rx);
  LegBuilder* beginRightLeg();
  LegBuilder* beginLeftLeg();
  Exoskeleton* build();
};

class LegBuilder{
private:
  ExoBuilder* return_context;
  int sign;
  State* states;
  std::vector<InputPort*> fsr_ports_begin;
  std::vector<InputPort*> torque_sensor_ports;
  std::vector<OutputPort*> motor_ports;
  std::vector<InputPort*> error_ports;
  std::vector<std::vector<InputPort*>> fsr_ports;
  std::vector<ImuPort*> imu_ports;
  std::vector<unsigned int> imu_address;
  std::vector<ControlModule*> controls;
public:
  LegBuilder(ExoBuilder* return_context);
  LegBuilder* addStateMachine(State* states);
  LegBuilder* addJoint(InputPort* torque_sensor_port, OutputPort* motor_port,
                       InputPort* error_port, ControlModule* module);
  LegBuilder* beginFSRGroup();
  LegBuilder* finishFSRGroup();
  LegBuilder* addFSR(InputPort* fsr_port);
  ExoBuilder* finishLeg();
  Leg* build();
};


#endif
