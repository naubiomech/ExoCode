#include "Shaping_Functions.h"
//Calc Sigmoid function and apply to the New point


double calculatePIDSetpointSigm(double New_PID_Setpoint, double Old_PID_Setpoint, double Ts, double exp_mult, int n_iter, int N)
{ // Makes the curve for your setpoint vs time look like the voltage time graph for a charging capacitor
  //n_iter tells you at which of the Nth sample you are not counting the zero
  //Ts sampling time in this case 0.001 with exp_mult=2000 it takes 6 milliseconds to rise from 0 to 1
  // it has to stop if N==n_iter
  //  N = round(1 / (Ts * exp_mult) * 10);
  //  if ((N % 2)) {
  //    N++;
  //  }
  double sig = 1 / (1 + exp(-exp_mult * ((-N / 2 + n_iter + 1)) * Ts));
  Current_PID_Setpoint = Old_PID_Setpoint + (New_PID_Setpoint - Old_PID_Setpoint) * sig;
  return Current_PID_Setpoint;
}

void PID_Sigm_Curve(Leg* leg){
  long sig_time = millis();                                                        //Start time for sig
  if ((sig_time - leg->sig_time_old) > 1)
  {
    leg->sig_time_old = sig_time;                                                  //??? records for next time this code runs??

    if ((abs(leg->New_PID_Setpoint - leg->PID_Setpoint) > 0.1) && (leg->sigm_done))
    { //if state machine transition has occured and the newsetpoint is greater than the setpoint
      leg->sigm_done = false;                                                   //Do not let the code enter this block, uhh until the setpoint transition has finished?
      n_iter = 0;
      int N_step;

      if (leg->state == LATE_STANCE) {
        N_step = N3;//12 * 10;                                             //Defines number of steps
      } else if (leg->state == SWING) {
        N_step = N1;
      }

      double exp_mult = round((10 / Ts) / (N_step - 1));                         //???

    }// end if sigm_done

    if (n_iter < N_step)
    {
      // Determines the new intermediate PID Setpoint
      leg->PID_Setpoint = calculatePIDSetpointSigm(New_PID_Setpoint, leg->Old_PID_Setpoint,
                                                   Ts, exp_mult, n_iter, N_step);
      n_iter++;
    } else {
      leg->sigm_done = true;
    }
  }
}
