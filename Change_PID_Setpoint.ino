double Change_PID_Setpoint_Trapz(double New_PID_Setpoint_l, double Current_PID_Setpoint, double Old_PID_Setpoint_l, double n_step) {

  double step_value = (New_PID_Setpoint_l - Old_PID_Setpoint_l) / n_step;

  if (fabs(New_PID_Setpoint_l - Current_PID_Setpoint) <= 0.01) {
    Current_PID_Setpoint = New_PID_Setpoint_l;
  }
  else {
    Current_PID_Setpoint += step_value;
  }

  return Current_PID_Setpoint;
}

//-------------------------

double Calc_KF(double New_PID_Setpoint_l, int test) {
  double KF = 1;

  if (test == 1) {
    //KF=1.1   if   0<=S<=5
    //KF=1.2   if    S<0 && S>5
    if (New_PID_Setpoint_l >= 0 && New_PID_Setpoint_l <= 5) {
      KF = 1.1;
    }
    else {
      KF = 1.2;
    }
  } else {

    //
    //  KF=1.2                         if S<=-2
    //KF=-0.05*S+1.1         if -2<S<=0
    //KF=1.1                         if  0<=S<=5
    //KF=0.05*(S-5)+1.1    if 5<S<=7
    //KF=1.2                         if S>7


    if (New_PID_Setpoint_l <= -2) {
      KF = 1.2;
    }
    else {
      if (New_PID_Setpoint_l > -2 && New_PID_Setpoint_l <= 0) {
        KF = 1.1 - 0.05 * New_PID_Setpoint_l;
      }
      else {
        if (New_PID_Setpoint_l > 0 && New_PID_Setpoint_l <= 5)  {
          KF = 1.1;
        }
        else {
          if (New_PID_Setpoint_l > 5 && New_PID_Setpoint_l <= 7) {
            KF = 1.1 + 0.05 * (New_PID_Setpoint_l - 5);
          }
          else {
            KF = 1.2;
          }
        }
      }
    }// end if New Pid <=-2

  }// end if test


  return KF;
}

//------------------------------------------------------

//Calc Sigmoid function and apply to the New point


double Change_PID_Setpoint_Sigm(double New_PID_Setpoint_l, double Current_PID_Setpoint, double Old_PID_Setpoint_l, double Ts_l, double exp_mult_l, int n_iter_l, int N_l) {
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

////-----------------------------------------------------------
//
//void Sigmoid(double New_PID_Setpoint_l,double Old_PID_Setpoint_l,double time_duration) { // this has to be called in the state machine, i.e. PID_setpoint= Sigmoid(Ankle....,time_duration)
//
//        
//  
//        double timeElapsed = 0;
//        int set_PID_setpoint = 1;
//        boolean sigm_done = false;
//
//          while (timeElapsed < time_duration) {
//            if (set_PID_setpoint) {
//              Old_PID_Setpoint_l = PID_Setpoint;
//              set_PID_setpoint = 0;
//            }
//
//            if (sigm_flag && not(sigm_done)) {
//              N = round(1 / (Ts * exp_mult) * 10);
//
//              if ((N % 2)) {
//                N++;
//              }
//
//              for (int n_iter = 0; n_iter < N; n_iter++) {
//                PID_Setpoint = Change_PID_Setpoint_Sigm(New_PID_Setpoint_l, PID_Setpoint, Old_PID_Setpoint_l, Ts, exp_mult, n_iter);
////                delay(20);
////                Serial.print("Not Zero : ");
////                Serial.println(PID_Setpoint);
//              }// end for
//
//              sigm_done = true;
//
//            }// end if sigm_done
//
////            delay(20);
////            Serial.print("Not Zero : ");
////            Serial.println(PID_Setpoint);
//          }//end while time_duration
//
//  return;
//}



