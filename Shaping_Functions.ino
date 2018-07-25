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

void PID_Sigm_Curve_RL()
{
  right_leg->sig_time = millis();                                                        //Start time for sig
  if (right_leg->sig_time - right_leg->sig_time_old > 1)
  {
    right_leg->sig_time_old = right_leg->sig_time;                                                  //??? records for next time this code runs??

    if (right_leg->sigm_flag)
    { //This is always true???
      if ((abs(right_leg->New_PID_Setpoint - right_leg->PID_Setpoint) > 0.1) && (right_leg->sigm_done))
      { //if state machine transition has occured and the newsetpoint is greater than the setpoint
        right_leg->sigm_done = false;                                                   //Do not let the code enter this block, uhh until the setpoint transition has finished?
        right_leg->n_iter = 0;

        if (right_leg->state == 3) {
          right_leg->N_step = right_leg->N3;//12 * 10;                                             //Defines number of steps
        }
        else if (right_leg->state == 2) {                                             //So I think this N_step business determines how quickly the PID Setpoint rises or falls
          right_leg->N_step = right_leg->N2;//6 * 10;
        }
        else if (right_leg->state == 1) {
          right_leg->N_step = right_leg->N1;
        }

        right_leg->exp_mult = round((10 / Ts) / (right_leg->N_step - 1));                         //???

      }// end if sigm_done

      if (right_leg->n_iter < right_leg->N_step)
      {
        // Determines the new intermediate PID Setpoint
        right_leg->PID_Setpoint = Change_PID_Setpoint_Sigm(right_leg->New_PID_Setpoint, right_leg->PID_Setpoint, right_leg->Old_PID_Setpoint, Ts, right_leg->exp_mult, right_leg->n_iter, right_leg->N_step);
        right_leg->n_iter++;                    //Takes in       goal Setpoint, instantaneous setpoint,   previous setpoint, time interval,    constant, our location along the x axis, length of x axis
      }
      if (right_leg->n_iter >= right_leg->N_step)
      {
        right_leg->sigm_done = true;
      }
    }// end if sigm
  }
}

void PID_Sigm_Curve_LL()
{
  left_leg->sig_time = millis();                                                        //Start time for sig
  if (left_leg->sig_time - left_leg->sig_time_old > 1)
  {
    left_leg->sig_time_old = left_leg->sig_time;                                                  //??? records for next time this code runs??

    if (left_leg->sigm_flag)
    { //This is always true???
      if ((abs(left_leg->New_PID_Setpoint - left_leg->PID_Setpoint) > 0.1) && (left_leg->sigm_done))
      { //if state machine transition has occured and the newsetpoint is greater than the setpoint

        left_leg->sigm_done = false;                                                   //Do not let the code enter this block, uhh until the setpoint transition has finished?
        left_leg->n_iter = 0;

        if (left_leg->state == 3) {
          left_leg->N_step = left_leg->N3;//12 * 10;                                             //Defines number of steps
        }
        else if (left_leg->state == 2) {                                             //So I think this N_step business determines how quickly the PID Setpoint rises or falls
          left_leg->N_step = left_leg->N2;//6 * 10;
        }
        else if (left_leg->state == 1) {
          left_leg->N_step = left_leg->N1;

        }
        left_leg->exp_mult = round((10 / Ts) / (left_leg->N_step - 1));                         //???
      }// end if sigm_done

      if (left_leg->n_iter < left_leg->N_step)
      {
        // Determines the new intermediate PID Setpoint
        left_leg->PID_Setpoint = Change_PID_Setpoint_Sigm(left_leg->New_PID_Setpoint, left_leg->PID_Setpoint, left_leg->Old_PID_Setpoint, Ts, left_leg->exp_mult, left_leg->n_iter, left_leg->N_step);
        left_leg->n_iter++;                    //Takes in       goal Setpoint, instantaneous setpoint,   previous setpoint, time interval,    constant, our location along the x axis, length of x axis
      }
      if (left_leg->n_iter >= left_leg->N_step)
      {
        left_leg->sigm_done = true;
      }
    }// end if sigm
  }
}



