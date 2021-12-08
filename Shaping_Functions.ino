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


//Optimization----------------------------------------------------------------------
double Change_PID_Setpoint_Spline(Leg* p_leg_l, double New_PID_Setpoint_l, double Current_PID_Setpoint, double Old_PID_Setpoint_l, double Ts_l, double exp_mult_l, int n_iter_l, int N_l, double T_l)
{
  //n_iter tells you at which of the Nth sample you are not counting the zero
  //Ts sampling time. In this case 0.001 with exp_mult=2000 results in 6 millisecond rise from 0 to 1

  Serial.print("Old T Setpoint: ");
  Serial.print(p_leg_l->Previous_T_Opt);
  Serial.print(" New T Setpoint: ");
  Serial.println(T_l);

  Serial.print("Old TRQ Setpoint: ");
  Serial.print(Old_PID_Setpoint_l);
  Serial.print(" New TRQ Setpoint: ");
  Serial.println(New_PID_Setpoint_l);

  p_leg_l->pf = New_PID_Setpoint_l;
  p_leg_l->p0 = Old_PID_Setpoint_l;
  p_leg_l->dp0 = 0;
  p_leg_l->dpf = 0;

  p_leg_l->c0 = p_leg_l->p0;
  p_leg_l->c1 = p_leg_l->dp0;
  p_leg_l->c2 = 2 * p_leg_l->ddp0;
  p_leg_l->c3 = -(20 * p_leg_l->c0 - 20 * p_leg_l->pf + 12 * T_l * p_leg_l->c1 - p_leg_l->ddpf * pow(T_l , 2) + 6 * pow(T_l , 2) * p_leg_l->c2 + 8 * p_leg_l->dpf * T_l) / (2 * pow(T_l, 3));
  p_leg_l->c4 = (15 * p_leg_l->c0 - 15 * p_leg_l->pf + 8 * T_l * p_leg_l->c1 - p_leg_l->ddpf * pow(T_l , 2) + 3 * pow(T_l, 2) * p_leg_l->c2 + 7 * p_leg_l->dpf * T_l) / (pow(T_l, 4));
  p_leg_l->c5 = -(12 * p_leg_l->c0 - 12 * p_leg_l->pf + 6 * T_l * p_leg_l->c1 - p_leg_l->ddpf * pow(T_l , 2) + 2 * pow(T_l , 2) * p_leg_l->c2 + 6 * p_leg_l->dpf * T_l) / (2 * pow(T_l , 5));

  double spl = p_leg_l->c5 * pow(n_iter_l * 0.002, 5) + p_leg_l->c4 * pow(n_iter_l * 0.002, 4) + p_leg_l->c3 * pow(n_iter_l * 0.002, 3) + p_leg_l->c2 * pow(n_iter_l * 0.002, 2) + p_leg_l->c1 * n_iter_l * 0.002 + p_leg_l->c0;
  if (spl > New_PID_Setpoint_l) {
    spl = New_PID_Setpoint_l;
  }

  Current_PID_Setpoint = spl;

  Serial.print(" Current PID Setpoint: ");
  Serial.println(Current_PID_Setpoint);
  return Current_PID_Setpoint;
}
//----------------------------------------------------------------------------------

//------
void PID_Sigm_Curve(Leg* leg) {
  leg->sig_time = millis();
  //Start time for sig
  if (leg->sig_time - leg->sig_time_old >= 2)
  {
    leg->sig_time_old = leg->sig_time;                                                  //??? records for next time this code runs??

    if (abs(leg->New_PID_Setpoint - leg->PID_Setpoint) > 0.1 &&  (leg->sigm_done))
    {
      leg->n_iter = 0;
      leg->sigm_done = false;                                                   //Do not let the code enter this block, until the setpoint transition has finished

      if (leg->state == 3) {
        leg->N_step = leg->N3;                                           //Defines number of steps
      }
      else if (leg->state == 2) {
        leg->N_step = leg->N2;
      }
      else if (leg->state == 1) {
        leg->N_step = leg->N1;
      }

      //Optimization-------------------------------
      if (Flag_HLO) {
        if (leg->state == 3) {
          leg->N_step = round(leg->T_Opt / 0.002);  // This is because we send a command every 2 ms
        }
        else {
          leg->N_step = 4;
        }
      }
      //------------------------------------------

      leg->exp_mult = round((10 / Ts) / (leg->N_step - 1));
    } // end if sigm_done

    //Optimization---------------------------------------------
    if (Flag_HLO && leg->FLAG_UPDATE_VALUES && not(leg->state == 3))
    {
      Serial.print("Update ");
      leg->Previous_Setpoint_Ankle = leg->Setpoint_Ankle;
      leg->Setpoint_Ankle = leg->Setpoint_Ankle_Opt;
      leg->Previous_T_Opt = leg->T_Opt;
      leg->T_Opt = max(0.1, leg->T_Opt_p * 1 * (leg->p_steps->plant_mean) * 0.001); //leg->T_Opt_p is the percentage of the stance phase
      Serial.print(" - Setpoint: ");
      Serial.print(leg->Setpoint_Ankle);
      Serial.print(" , Mean Time Stance Phase: ");
      Serial.print((leg->p_steps->plant_mean) * 0.001);
      Serial.print(" , Rise Time Optimization: ");
      Serial.println(leg->T_Opt);
      leg->FLAG_UPDATE_VALUES = false;
    }
    //---------------------------------------------------------
    
    if (leg->sigm_done == false && leg->n_iter < leg->N_step)
    {
      //Optimization--------------------------------------------------------
      if (Flag_HLO && leg->state == 3) {
        leg->PID_Setpoint = Change_PID_Setpoint_Spline(leg, leg->New_PID_Setpoint, leg->PID_Setpoint, leg->Old_PID_Setpoint, Ts, leg->exp_mult, leg->n_iter, leg->N_step, leg->T_Opt_Setpoint);
      }
      //--------------------------------------------------------------------
      else {
        // Determines the new intermediate PID Setpoint
        leg->PID_Setpoint = Change_PID_Setpoint_Sigm(leg->New_PID_Setpoint, leg->PID_Setpoint, leg->Old_PID_Setpoint, Ts, leg->exp_mult, leg->n_iter, leg->N_step);
      }
      leg->n_iter++;                    //Takes in       goal Setpoint, instantaneous setpoint,   previous setpoint, time interval,    constant, our location along the x axis, length of x axis
    }


  }
}
