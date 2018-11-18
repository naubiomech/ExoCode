#ifndef STATE_MACHINE_HEADER
#define STATE_MACHINE_HEADER
#include "Utils.hpp"


enum StateType {SWING = 1, LATE_STANCE = 3};

class Leg;
class State{
private:
  Timer* state_time = new Timer();
  State* next_state;
protected:
  Leg* leg;
public:
  double getStateTime();
  State* changeState();
  void setNextState(State* next_phase);
  void setContext(Leg* leg);
  virtual void triggerStart();
  virtual void triggerEnd();
  virtual void run()=0;
  virtual StateType getStateType();
};

class SwingState : public State{
public:
  void run();
};

class LateStanceState : public State{
public:
  void triggerStart();
  void run();
  void triggerEnd();
};

#endif
