//Paramenters used to shape the reference
double Ts = 0.001;
double exp_mult_RL = 1500.0;
boolean sigm_flag_RL = true;
double exp_mult_LL = 1500.0;
boolean sigm_flag_LL = true;

double New_PID_Setpoint_RL = 0.0;
double Old_PID_Setpoint_RL = 0.0;
double New_PID_Setpoint_LL = 0.0;
double Old_PID_Setpoint_LL = 0.0;

// the smoothing value, i.e. the sigmoind number of steps as a function of the EXO state
double N3 = 500;
double N2 = 4;
double N1 = 4;

//for the left leg
double N3_LL = N3;
double N2_LL = N2;
double N1_LL = N1;

//for the right leg
double N3_RL = N3;
double N2_RL = N2;
double N1_RL = N1;

long sig_time_RL = 0;
long sig_time_old_RL = 0;
long sig_time_LL = 0;
long sig_time_old_LL = 0;

int change = 1;
int just_first_time = 1;
int n_iter_RL, N_step_RL;
int n_iter_LL, N_step_LL;

boolean sigm_done_LL = true;
boolean sigm_done_RL = true;
