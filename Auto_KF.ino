#include "Auto_KF.h"
#include "utils.h"

void Auto_KF_motor_Late_stance(Average* error_average, double pid_setpoint, double motor_input){
  error_average->update(pid_setpoint - motor_input);
}

void Auto_KF_motor_Swing(Average* error_average, double KF, Clamp* kf_clamp){
  double err = error_average->getAverage();
  error_average->reset();

  if (motor->ERR > max_ERR) {
    motor->KF += 0.05;
  }
  else if (motor->ERR < min_ERR) {
    motor->KF -= 0.05;
  }

  return kf_clamp->clamp(KF);
}
