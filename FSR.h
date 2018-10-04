#ifndef FSR_HEADER
#define FSR_HEADER

class FSR{
public:
  unsigned int pin;

  double force;

  double fsr = 0;
  double fsr_peak_ref = 0;
  double fsr_percent_thresh = 0.9;
  int address_FSR;
  int baseline_address;
  double baseline_value;
};

class FSRGroup{
private:
  int fsr_count;
  FSR* fsrs[];
public:
  FSRGroup(FSR* fsrs, int fsr_count);

  double getMeasure();
  double getForce();

  double force;

  double fsr_Combined_peak_ref;
  double FSR_Ratio;
  double Max_FSR_Ratio;
  int FSR_baseline_FLAG = 0;
};
#endif
