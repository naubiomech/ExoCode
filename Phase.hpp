#ifndef PHASE_HEADER
#define PHASE_HEADER
#include "Utils.hpp"

enum PhaseType {PLANTER, DORSI};
class Leg;

class Phase{
private:
  Timer* phase_time = new Timer();
  MovingAverage* phase_time_average = new MovingAverage(4);
  Phase* next_phase;
protected:
  Leg* leg;
public:
  double getPhaseTime();
  double getAverageTime();
  Phase* changePhase();
  void setNextPhase(Phase* next_phase);
  void setContext(Leg* leg);
  virtual void triggerStart();
  virtual void triggerEnd();
  virtual void run()=0;
  virtual PhaseType getPhaseType();
};

class DorsiPhase: public Phase {
public:
  void run();
};

class PlanterPhase: public Phase{
public:
  void triggerStart();
  void run();
  void triggerEnd();
};
#endif
