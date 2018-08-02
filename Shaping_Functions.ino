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
  sig_time_RL = millis();                                                        //Start time for sig
  if (sig_time_RL - sig_time_old_RL > 1)
  {
    sig_time_old_RL = sig_time_RL;                                                  //??? records for next time this code runs??

    if (sigm_flag_RL)
    { //This is always true???
      if ((abs(New_PID_Setpoint_RL - PID_Setpoint_RL) > 0.1) && (sigm_done_RL))
      { //if state machine transition has occured and the newsetpoint is greater than the setpoint
        //if (1) {
        sigm_done_RL = false;                                                   //Do not let the code enter this block, uhh until the setpoint transition has finished?
        n_iter_RL = 0;

        if (R_state == 3) {
          N_step_RL = N3_RL;//12 * 10;                                             //Defines number of steps
        }
        else if (R_state == 2) {                                             //So I think this N_step business determines how quickly the PID Setpoint rises or falls
          N_step_RL = N2_RL;//6 * 10;
        }
        else if (R_state == 1) {
          N_step_RL = N1_RL;
        }

        exp_mult_RL = round((10 / Ts) / (N_step_RL - 1));                         //???

      }// end if sigm_done

      if (n_iter_RL < N_step_RL)
      {
        // Determines the new intermediate PID Setpoint
        PID_Setpoint_RL = Change_PID_Setpoint_Sigm(New_PID_Setpoint_RL, PID_Setpoint_RL, Old_PID_Setpoint_RL, Ts, exp_mult_RL, n_iter_RL, N_step_RL);
        n_iter_RL++;                    //Takes in       goal Setpoint, instantaneous setpoint,   previous setpoint, time interval,    constant, our location along the x axis, length of x axis
      }
      if ((n_iter_RL >= N_step_RL)&& (abs(New_PID_Setpoint_RL - PID_Setpoint_RL) <= 0.1))
      {
        sigm_done_RL = true;
      }
    }// end if sigm
  }
}

void PID_Sigm_Curve_LL()
{
  sig_time_LL = millis();                                                        //Start time for sig
  if (sig_time_LL - sig_time_old_LL > 1)
  {
    sig_time_old_LL = sig_time_LL;                                                  //??? records for next time this code runs??

    if (sigm_flag_LL)
    { //This is always true???
      if ((abs(New_PID_Setpoint_LL - PID_Setpoint_LL) > 0.1) && (sigm_done_LL))
      { //if state machine transition has occured and the newsetpoint is greater than the setpoint
        //        Serial.print("cond ");
        //        Serial.println((abs(New_PID_Setpoint_LL - PID_Setpoint_LL) > 0.1));
        sigm_done_LL = false;                                                   //Do not let the code enter this block, uhh until the setpoint transition has finished?
        n_iter_LL = 0;

        if (L_state == 3) {
          N_step_LL = N3_LL;//12 * 10;                                             //Defines number of steps
        }
        else if (L_state == 2) {                                             //So I think this N_step business determines how quickly the PID Setpoint rises or falls
          N_step_LL = N2_LL;//6 * 10;
        }
        else if (L_state == 1) {
          N_step_LL = N1_LL;

        }
        exp_mult_LL = round((10 / Ts) / (N_step_LL - 1));                         //???
      }// end if sigm_done

      if (n_iter_LL < N_step_LL)
      {
        // Determines the new intermediate PID Setpoint
        PID_Setpoint_LL = Change_PID_Setpoint_Sigm(New_PID_Setpoint_LL, PID_Setpoint_LL, Old_PID_Setpoint_LL, Ts, exp_mult_LL, n_iter_LL, N_step_LL);
        n_iter_LL++;                    //Takes in       goal Setpoint, instantaneous setpoint,   previous setpoint, time interval,    constant, our location along the x axis, length of x axis
      }
      if ((n_iter_LL >= N_step_LL) && (abs(New_PID_Setpoint_LL - PID_Setpoint_LL) <= 0.1))
      {
        sigm_done_LL = true;
      }
    }// end if sigm
  }
}



