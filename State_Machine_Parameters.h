int R_state = 1;
int L_state = 1;
int R_state_old = 1;
int L_state_old = 1;
int state_count_RL_13 = 0;                                            //These values are included to count how many times the the transition between states occurs
int state_count_RL_31 = 0;                                            //The intent is to not let the code to transition states if an outlier sensor measurement were to occur
int state_count_LL_13 = 0;
int state_count_LL_31 = 0;

double R_state_3_start_time = 0;
double R_state_1_start_time = 0;
double R_start_from_1 = 0;
double R_start_from_3 = 0;

double L_state_3_start_time = 0;
double L_state_1_start_time = 0;
double L_start_from_1 = 0;
double L_start_from_3 = 0;


double state_counter_th = 8;
double R_start_time = 0;
double L_start_time = 0;
double step_time_length = 250;