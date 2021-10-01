/* This file declares functions that are used to change the state of the motors. */

#ifndef M_UTIL_H
#define M_UTIL_H

#include "akxMotor.h"

inline void change_motor_state(akxMotor* akMotor, bool to)
{
  motors_on = to;
  flag_motor_error_check = to;
  flag_auto_KF = to;
  //digitalWrite(onoff, to);
  akMotor->setMotorState(L_ID, to);
  akMotor->setMotorState(R_ID, to);
}

inline void change_motor_stateless(akxMotor* akMotor, bool to) {
  flag_motor_error_check = to;
  flag_auto_KF = to;
  //digitalWrite(onoff, to);
  akMotor->setMotorState(L_ID, to);
  akMotor->setMotorState(R_ID, to);
}


#endif
