#include "pid.h"
#include "Leg.h"
#include "Board.h"

void pid(Leg* leg, double trq, double stability){

  PID* pid;
  double input;

  if (IMU_ENABLED && leg->state == 3 && Trq_time_volt == 2){
    input = stability * leg->Prop_Gain;
    pid = &(leg->balance_pid);
  } else{
    input = trq;
    pid = &(leg->ankle_pid);

    if ((abs(input) > 25))
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

  leg->Input = input;
  pid->Compute_KF(leg->KF);
  leg->Vol = leg->Output + zero; //need to map
  analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
}
