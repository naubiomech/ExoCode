
class FSR{
public:
	double FSR_Average_array[dim_FSR] = {0};
	double Curr_FSR= 0;
	volatile double FSR_Average;
	volatile double FSR_Balance_Baseline;
	double Curr;
	unsigned int fsr_pin;
	double fsr = 0;
	double fsr_peak_ref = 0;
	double fsr_percent_thresh = 0.9;
	int address_FSR;
	int baseline_address;
	double baseline_value;
}

class FSRGroup{
	FSR fsrs[];
	volatile double FSR_Combined_Average;
	double fsr_Combined_peak_ref;
	double Curr_Combined;
	double FSR_Ratio;
	double Max_FSR_Ratio;
	int FSR_baseline_FLAG = 0;

}
