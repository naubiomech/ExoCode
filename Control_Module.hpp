#ifndef CONTROL_MODULE_HEADER
#define CONTROL_MODULE_HEADER
#include "Arduino.hpp"
#include "States.hpp"
#include "Control_Algorithms.hpp"
#include "Shaping_Functions.hpp"
#include "Utils.hpp"
#include "Linked_List.hpp"

class ControlModule{
private:
  PID* pid;
  ShapingFunction* shaping_function;
  double clamp_setpoint(double raw_setpoint);
  ControlAlgorithm* current_algorithm;
  bool adjust_shaping_for_time = false;
  Clamp* kf_clamp;
  Clamp* adjust_shaping_for_time_clamp;

  double KF = 1;
  double current_pid_setpoint = 0;
  double pid_input = 0;
  double pid_output = 0;
  double last_control_pid_setpoint;
  RunningAverage* error_average;
  double iter_time_percentage = 0.5;

  ControlAlgorithm* getControlAlgorithm(StateID state_id);
  double getSetpoint(double fsr_percentage, double fsr_max_percentage);
  double shapeSetpoint(double new_setpoint);
  double runPID(double torque_input, double kf, double pid_setpoint);

public:
  ControlModule(ControlAlgorithm* state_machine, StateID starting_state);
  ~ControlModule();
  void setControlStateMachine(ControlAlgorithm* state_machine, StateID starting_state);
  void setToZero();
  void setDesiredSetpoint(double setpoint);
  void changeState(StateID state_id);
  double getControlAdjustment(double torque_input, double fsr_percentage, double fsr_max_percentage);
  void resetStartingParameters();
  void adjustShapingForTime(double planter_time);
  void changeControl(StateID state_id);
  void updateKFPIDError(double torque);
  void applyAutoKF();
  double getLastSetpoint();
};

class ControlModuleBuilder{
private:
  LinkedList<StateID> states;
  LinkedList<ControlAlgorithmType> control_types;
public:
  ControlModuleBuilder* addState(StateID state, ControlAlgorithmType control_type);
  ControlModule* build(StateID starting_state);
  ControlAlgorithm* createControlAlgorithm(ControlAlgorithmType type, StateID state_id);
};
#endif
