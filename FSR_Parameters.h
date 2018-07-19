// To set FSR bias and to identify the states
double fsrLongCurrent = 0;
double fsrShortCurrent = 0;
//const unsigned int fsr_sense_Long = 2;
//const unsigned int fsr_sense_Short = 3;

const unsigned int fsr_sense_Left_Heel = A14;
const unsigned int fsr_sense_Left_Toe = A15;
const unsigned int fsr_sense_Right_Heel = A12;
const unsigned int fsr_sense_Right_Toe = A13;

double fsr_Left_Heel = 0;
double fsr_Left_Toe = 0;
double fsr_Right_Heel = 0;
double fsr_Right_Toe = 0;
double fsr_Left_Heel_thresh = 0;                      //These "thresh" values are what I called the "cal" values in previous versions
double fsr_Left_Toe_thresh = 0;                       //Also known asthe values to determine when to transition states
double fsr_Right_Heel_thresh = 0;
double fsr_Right_Toe_thresh = 0;

double doubleFSR = 0;
int intFSR = 0;

double fsr_percent_thresh_Left_Heel = .9;              //These are the percentage of the threshold values where the code will say
double fsr_percent_thresh_Left_Toe = .9;               //if it is "this" percent of our calibration/threshold value we are "close enough" to change the state
double fsr_percent_thresh_Right_Heel = .9;             //These are the percentage of the threshold values where the code will say
double fsr_percent_thresh_Right_Toe = .9;              //if it is "this" percent of our calibration/threshold value we are "close enough" to change the state
