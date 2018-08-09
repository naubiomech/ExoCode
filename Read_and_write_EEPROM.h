#ifndef READ_AND_WRITE_EEPROM_HEADER
#define READ_AND_WRITE_EEPROM_HEADER

int check_torque_bias(int address_torque_l);

int write_torque_bias(int address_torque_l, double torque_adj);

double read_torque_bias(int address_torque_l);

int clean_torque_bias(int address_torque_l);

int check_FSR_values(int address_FSR_l);

int write_FSR_values(int address_FSR_l, double FSR_adj);

double read_FSR_values(int address_FSR_l);

int clean_FSR_values(int address_FSR_l);

// write and read values in EEPROM also per KF, smoothing ,PID and FSR params
// Since it is really big update I would suggest to do it before the trial not in the middle
int write_EXP_parameters(int address_params_l);

int check_EXP_parameters(int address_params_l);

double read_param(int address_l);

int read_all_params(int address_params_l);

int clean_EXP_Parameters(int address_params_l);

int write_baseline(int address_baseline_l, double baseline_val);

double read_baseline(int address_baseline_l);

#endif
