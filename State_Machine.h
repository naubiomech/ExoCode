#ifndef STATE_MACHINE_HEADER
#define STATE_MACHINE_HEADER

// TODO Change this to use an enum
const int SWING = 1;
const int LATE_STANCE = 3;

#include "Leg.h"

void state_machine(Leg* leg);

#endif
