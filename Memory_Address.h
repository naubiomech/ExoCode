//To store in memory. These are the address of the EEPROM of the Teensy
int address_torque_LL = 0;
int address_torque_RL = 9; // 9 beacuse it is a char and a double
int address_FSR_LL = 18;
int address_FSR_RL = 36;

int address_params = 54;

int L_baseline_address = address_params + 105 + 5; //add five just to be sure
int R_baseline_address = L_baseline_address + 9;

int flag_save_EEPROM = 0;
