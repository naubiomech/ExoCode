#include "pid.h"
#include "Leg.h"
#include "Board.h"

void pid(Leg* leg, double meas_trq, double meas_IMU) {

  PID* pid;
  double PID_ref;

  if (IMU_ENABLED && leg->state == LATE_STANCE && Trq_time_volt == 2) {
    if (meas_IMU >= 45)
    {
      meas_IMU = 45;
    }
    else if (meas_IMU <= -45) {
      meas_IMU = -45;
    }
    PID_ref = 0;
    leg->PID_Setpoint = PID_ref;
    leg->Input = meas_IMU * leg->Prop_Gain; // this is totally new, we have to test!
    pid = &(leg->balance_pid);
  } else {
    PID_ref = leg->PID_Setpoint;
    leg->Input = meas_trq;
    pid = &(leg->ankle_pid);

    if ((abs(meas_trq) > 25))
    {
      leg->KF = 0;
      double old_L_state_L = leg->state;
      leg->state = 9;
      send_data_message_wc();

      digitalWrite(MOTOR_ENABLE_PIN, LOW);
      stream = 0;
      digitalWrite(LED_PIN, LOW);
      leg->state = old_L_state_L;

    }
  }

  pid->Compute_KF(leg->KF);

  //This can be used as alternative to the previous gain (see up)
  if (IMU_ENABLED && leg->state == LATE_STANCE && Trq_time_volt == 2) {
    leg->Output *= leg->Prop_Gain;
    if (leg->Output >= 1500) leg->Output = 1500;
    if (leg->Output <= -1500) leg->Output = -1500;
  }

  leg->Vol = leg->Output + leg->zero; //need to map

  analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
}
