#include "Auto_KF.h"
#include "Parameters.h"
#include "Utils.h"

void Auto_KF_motor_Late_stance(Average* error_average, double pid_setpoint, double motor_input){
  error_average->update(pid_setpoint - motor_input);
}

double Auto_KF_motor_Swing(Average* error_average, double KF, Clamp* kf_clamp){
  double err = error_average->getAverage();
  error_average->reset();

  if (err > max_ERR) {
    KF += 0.05;
  }
  else if (err < min_ERR) {
    KF -= 0.05;
  }

  return kf_clamp->clamp(KF);
}
