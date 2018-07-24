#include "Board.h"
// To set FSR bias and to identify the states
double fsrLongCurrent = 0;
double fsrShortCurrent = 0;

// Single board small
const unsigned int fsr_sense_Left_Heel = FSR_SENSE_LEFT_HEEL_PIN;
const unsigned int fsr_sense_Left_Toe = FSR_SENSE_LEFT_TOE_PIN;
const unsigned int fsr_sense_Right_Heel = FSR_SENSE_RIGHT_HEEL_PIN;
const unsigned int fsr_sense_Right_Toe = FSR_SENSE_RIGHT_TOE_PIN;

double fsr_Left_Heel = 0;
double fsr_Left_Toe = 0;
double fsr_Right_Heel = 0;
double fsr_Right_Toe = 0;
double fsr_Left_Heel_peak_ref = 0;                      //These "thresh" values are what I called the "cal" values in previous versions
double fsr_Left_Toe_peak_ref = 0;                       //Also known asthe values to determine when to transition states
double fsr_Right_Heel_peak_ref = 0;
double fsr_Right_Toe_peak_ref = 0;

double doubleFSR = 0;
int intFSR = 0;

double fsr_percent_thresh_Left_Heel = .9;              //These are the percentage of the threshold values where the code will say
double fsr_percent_thresh_Left_Toe = .9;               //if it is "this" percent of our calibration/threshold value we are "close enough" to change the state
double fsr_percent_thresh_Right_Heel = .9;             //These are the percentage of the threshold values where the code will say
double fsr_percent_thresh_Right_Toe = .9;              //if it is "this" percent of our calibration/threshold value we are "close enough" to change the state

int FSR_FIRST_Cycle = 1;
int FSR_CAL_FLAG = 0;
int FSR_baseline_FLAG_Left = 0;
int FSR_baseline_FLAG_Right = 0;
int * p_FSR_baseline_FLAG_Right = &FSR_baseline_FLAG_Right;
int * p_FSR_baseline_FLAG_Left = &FSR_baseline_FLAG_Left;

double Curr_Left_Toe;
double Curr_Left_Heel;
double Curr_Right_Toe;
double Curr_Right_Heel;

double base_1, base_2;

double FSR_Sensors_type = 40;
