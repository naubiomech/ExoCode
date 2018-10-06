#ifndef STEADY_STATE_HEADER
#define STEADY_STATE_HEADER

// If the participant keeps the state more than 3 seconds, the torque reference is set to 0
void set_to_zero_if_leg_in_steady_state(Leg* leg);

#endif
