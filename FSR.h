#ifndef FSR_HEADER
#define FSR_HEADER

class FSR{
private:
  bool isCalibrating = false;
  unsigned long fsrCalibrationStartTime;
  void startCalibration();
  void updateCalibration();
  void endCalibration();

public:
  FSR(int pin);
  double calibrate();
  void measureForce();
  double getForce();
  double getBalanceReference();

  unsigned int pin;

  double force = 0;
  double peak_force = 0;

  double fsr = 0;
  int address_FSR;
  int baseline_address;
  double baseline_value;
};

class FSRGroup{
private:
  int fsr_count;
  double calibration_peak;
  FSR** fsrs;

  void startCalibration();
  void updateCalibration();

public:
  FSRGroup(FSRPins* fsr_pins, int fsr_count);

  void measureForce();
  double getForce();
  void calibrate();
  double getBalanceReference();

  double force;

  double fsr_Combined_peak_ref;
  double fsr_percent_thresh = 0.9;
  double FSR_Ratio;
  double Max_FSR_Ratio;
  int FSR_baseline_FLAG = 0;
};
#endif
