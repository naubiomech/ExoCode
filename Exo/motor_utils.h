/* This file declares functions that are used to change the state of the motors. */

#ifndef M_UTIL_H
#define M_UTIL_H

inline void change_motor_state(bool to)
{
  motors_on = to;
  flag_motor_error_check = to;
  flag_auto_KF = to;
  digitalWrite(onoff, to);
}

inline void change_motor_stateless(bool to) {
  flag_motor_error_check = to;
  flag_auto_KF = to;
  digitalWrite(onoff, to);
}


#endif
