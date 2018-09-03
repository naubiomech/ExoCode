#include "Auto_KF.h"
#include "Leg.h"

void sub_Auto_KF(Leg* leg){
  // take error in state 3
  if (leg->state == LATE_STANCE) {
    //    leg->Input is the average of the measured torque
    //    leg->PID_Stepoint is the reference
    Serial.print(" Leg Error ");
    Serial.println(leg->PID_Setpoint - leg->Input );
    leg->ERR += (leg->PID_Setpoint - leg->Input );
    leg->count_err++;
  }

  if (leg->state == SWING) {

    leg->ERR = leg->ERR / leg->count_err;
    if ((leg->count_err != 0)) {
      Serial.print("Leg ERR ");
      Serial.println(leg->ERR);
    }
    else {

    }
    leg->count_err = 0;



    if (leg->ERR > max_ERR) {
      leg->KF += 0.05;
    }
    else if (leg->ERR < min_ERR) {
      leg->KF -= 0.05;
    }
    else {}

    if (leg->KF >= leg->max_KF)
      leg->KF = leg->max_KF;
    else if (leg->KF <= leg->min_KF)
      leg->KF = leg->min_KF;
    else {}

    Serial.print("New leg->KF ");
    Serial.println(leg->KF);
    leg->ERR = 0;
  }

}

void Auto_KF() {
  sub_Auto_KF(left_leg);
  sub_Auto_KF(right_leg);
}
