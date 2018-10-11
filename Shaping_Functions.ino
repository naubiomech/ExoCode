#include "Shaping_Functions.h"
//Calc Sigmoid function and apply to the New point


double calculatePIDSetpointSigm(double New_PID_Setpoint, double Old_PID_Setpoint,
                                double Ts, double exp_mult, int n_iter, int N)
{ // Makes the curve for your setpoint vs time look like the voltage time graph for a charging capacitor
  //leg->n_iter tells you at which of the Nth sample you are not counting the zero
  //Ts sampling time in this case 0.001 with exp_mult=2000 it takes 6 milliseconds to rise from 0 to 1
  // it has to stop if N==leg->n_iter
  //  N = round(1 / (Ts * exp_mult) * 10);
  //  if ((N % 2)) {
  //    N++;
  //  }
  double sig = 1 / (1 + exp(-exp_mult * ((-1 * N / 2 + n_iter + 1)) * Ts));
  double Current_PID_Setpoint = Old_PID_Setpoint + (New_PID_Setpoint - Old_PID_Setpoint) * sig;
  return Current_PID_Setpoint;
}
