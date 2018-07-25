#include "Board.h"
// To set FSR bias and to identify the states
double fsrLongCurrent = 0;
double fsrShortCurrent = 0;

// Single board small

double doubleFSR = 0;
int intFSR = 0;

int FSR_FIRST_Cycle = 1;
int FSR_CAL_FLAG = 0;

double base_1, base_2;

double FSR_Sensors_type = 40;
