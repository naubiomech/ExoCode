#include "Auto_KF.h"
#include "Leg.h"

void Auto_KF_leg(Leg* leg){
  // take error in state 3
  switch(leg->state){
  case LATE_STANCE:
    Auto_KF_leg_Late_stance(leg);
    break;
  case SWING:
    Auto_KF_leg_Swing(leg);
    break;
  }
}

void Auto_KF_leg_Late_stance(Leg* leg){
  Serial.print(" Leg Error ");
  Serial.println(leg->PID_Setpoint - leg->Input );
  leg->ERR += (leg->PID_Setpoint - leg->Input );
  leg->count_err++;
}

void Auto_KF_leg_Swing(Leg* leg){
  leg->ERR = leg->ERR / leg->count_err;
  if ((leg->count_err != 0)) {
    Serial.print("Leg ERR ");
    Serial.println(leg->ERR);
  }
  leg->count_err = 0;

  if (leg->ERR > max_ERR) {
    leg->KF += 0.05;
  }
  else if (leg->ERR < min_ERR) {
    leg->KF -= 0.05;
  }

  if (leg->KF >= leg->max_KF){
    leg->KF = leg->max_KF;
  }else if (leg->KF <= leg->min_KF){
    leg->KF = leg->min_KF;
  }

  Serial.print("New leg->KF ");
  Serial.println(leg->KF);
  leg->ERR = 0;
}

void Auto_KF() {
  Auto_KF_leg(left_leg);
  Auto_KF_leg(right_leg);
}
