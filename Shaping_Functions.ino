#include "Shaping_Functions.h"
//Calc Sigmoid function and apply to the New point


double Change_PID_Setpoint_Sigm(double New_PID_Setpoint_l, double Current_PID_Setpoint, double Old_PID_Setpoint_l, double Ts_l, double exp_mult_l, int n_iter_l, int N_l)
{ // Makes the curve for your setpoint vs time look like the voltage time graph for a charging capacitor
  //n_iter tells you at which of the Nth sample you are not counting the zero
  //Ts sampling time in this case 0.001 with exp_mult=2000 it takes 6 milliseconds to rise from 0 to 1
  // it has to stop if N==n_iter
  //  N_l = round(1 / (Ts_l * exp_mult_l) * 10);
  //  if ((N_l % 2)) {
  //    N_l++;
  //  }
  double sig = 1 / (1 + exp(-exp_mult_l * ((-N_l / 2 + n_iter_l + 1)) * Ts_l));
  Current_PID_Setpoint = Old_PID_Setpoint_l + (New_PID_Setpoint_l - Old_PID_Setpoint_l) * sig;
  return Current_PID_Setpoint;
}

void PID_Sigm_Curve(Leg* leg){
  leg->sig_time = millis();                                                        //Start time for sig
  if (leg->sig_time - leg->sig_time_old > 1)
  {
    leg->sig_time_old = leg->sig_time;                                                  //??? records for next time this code runs??

    if (leg->sigm_flag)
    { //This is always true???
      if ((abs(leg->New_PID_Setpoint - leg->PID_Setpoint) > 0.1) && (leg->sigm_done))
      { //if state machine transition has occured and the newsetpoint is greater than the setpoint
        leg->sigm_done = false;                                                   //Do not let the code enter this block, uhh until the setpoint transition has finished?
        leg->n_iter = 0;

        if (leg->state == 3) {
          leg->N_step = leg->N3;//12 * 10;                                             //Defines number of steps
        }
        else if (leg->state == 2) {                                             //So I think this N_step business determines how quickly the PID Setpoint rises or falls
          leg->N_step = leg->N2;//6 * 10;
        }
        else if (leg->state == 1) {
          leg->N_step = leg->N1;
        }

        leg->exp_mult = round((10 / Ts) / (leg->N_step - 1));                         //???

      }// end if sigm_done

      if (leg->n_iter < leg->N_step)
      {
        // Determines the new intermediate PID Setpoint
        leg->PID_Setpoint = Change_PID_Setpoint_Sigm(leg->New_PID_Setpoint, leg->PID_Setpoint, leg->Old_PID_Setpoint, Ts, leg->exp_mult, leg->n_iter, leg->N_step);
        leg->n_iter++;                    //Takes in       goal Setpoint, instantaneous setpoint,   previous setpoint, time interval,    constant, our location along the x axis, length of x axis
      }
      if (leg->n_iter >= leg->N_step)
      {
        leg->sigm_done = true;
      }
    }// end if sigm
  }
}

void PID_Sigm_Curve_RL()
{
  PID_Sigm_Curve(right_leg);
}

void PID_Sigm_Curve_LL()
{
  PID_Sigm_Curve(left_leg);
}
