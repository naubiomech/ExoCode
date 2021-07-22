#ifndef resetMotorIfError_HEADER
#define resetMotorIfError_HEADER

#define ERROR_COUNT_THRESH  2
#define RESET_COUNT_THRESH  16 / CONTROL_TIME_STEP
#define WAIT_COUNT_THRESH   40 / CONTROL_TIME_STEP

void resetMotorIfError();
int reset_count = 0;

/*
bool turn_off_drivers = false;
int motor_error_counter = 0;
bool flag_enable_catch_error = true;
bool count_time_drivers_are_off = true;
int time_drivers_are_off = 0;
int wait_to_allow_for_error_check = 0;
bool count_wait_to_allow_for_error_check = false;
void resetMotorIfError();
int reset_count = 0;
*/


#endif
