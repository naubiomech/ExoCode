#ifndef EXO_BUILDER_HEADER
#define EXO_BUILDER_HEADER
#include "Port.hpp"
#include "Exoskeleton.hpp"
#include "Board.hpp"
#include "Linked_List.hpp"


class ExoDirector{
public:
  virtual Exoskeleton* build(Board* board) = 0;
};

class QuadExoDirector:ExoDirector{
public:
  Exoskeleton* build(Board* board);
};

class LegBuilder;
class ExoBuilder{
private:
  RxPort* rx;
  TxPort* tx;
  OutputPort* led_port;
  OutputPort* motor_enable_port;
  LegBuilder* right_builder;
  LegBuilder* left_builder;
public:
  ExoBuilder* addTransceiver(TxPort* tx, RxPort* rx);
  ExoBuilder* addMotorEnable(OutputPort* motor_enable_port);
  ExoBuilder* addLedPort(OutputPort* led_port);
  LegBuilder* beginRightLeg();
  LegBuilder* beginLeftLeg();
  Exoskeleton* build();
};

class LegBuilder{
private:
  ExoBuilder* return_context;
  int sign;
  State* states;
  LinkedList<InputPort*> fsr_ports_begin;
  LinkedList<InputPort*> torque_sensor_ports;
  LinkedList<OutputPort*> motor_ports;
  LinkedList<InputPort*> error_ports;
  LinkedList<LinkedList<InputPort*>> fsr_ports;
  LinkedList<ImuPort*> imu_ports;
  LinkedList<unsigned int> imu_address;
  LinkedList<ControlModule*> controls;
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
