#ifndef STEADY_STATE_HEADER
#define STEADY_STATE_HEADER

// If the participant keeps the state more than 3 seconds, the torque reference is set to 0
void set_2_zero_if_steady_state();

#endif
