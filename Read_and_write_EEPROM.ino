#include "Read_and_write_EEPROM.h"
// Functions to check the memory values and to read them

int check_torque_bias(int address_torque_l)
{
  byte value = EEPROM.read(address_torque_l);
  if (char(value) == 'y')
  {
    Serial.println(" Torque value is present in memory ");
    return 1;
  }
  Serial.println(" Torque value is not present in memory ");
  return 0;
};

int write_torque_bias(int address_torque_l, double torque_adj)
{
  EEPROM.put(address_torque_l, 'y');
  address_torque_l++;
  EEPROM.put(address_torque_l, torque_adj);
  return 1;
}

double read_torque_bias(int address_torque_l)
{
  address_torque_l += 1;
  double val_t = 0;
  byte array_values[8];
  byte* p_array = array_values;
  for (unsigned int i = 0; i < sizeof(double); i++)
  {
    *(p_array + i) = EEPROM.read(address_torque_l + i);
  }
  memcpy(&val_t, &array_values, 8);
  return val_t;
}

int clean_torque_bias(int address_torque_l)
{
  for ( unsigned int i = address_torque_l ; i < (sizeof(double) + sizeof(char) + address_torque_l) ; i++ )
  {
    EEPROM.write(i, 0);
  }
  return 1;
}


int check_FSR_values(int address_FSR_l)
{
  byte value = EEPROM.read(address_FSR_l);
  if (char(value) == 'y') {
    Serial.println(" FSR values are present in memory ");
    return 1;
  }
  Serial.println(" FSR values are not present in memory ");
  return 0;
}

int write_FSR_values(int address_FSR_l, double FSR_adj)
{
  EEPROM.put(address_FSR_l, 'y');
  address_FSR_l++;
  EEPROM.put(address_FSR_l, FSR_adj);
  return 1;
}

double read_FSR_values(int address_FSR_l)
{
  address_FSR_l += 1;
  double val_t = 0;
  byte array_values[8];
  byte* p_array = array_values;
  for (unsigned int i = 0; i < sizeof(double); i++)
  {
    *(p_array + i) = EEPROM.read(address_FSR_l + i);
  }
  memcpy(&val_t, &array_values, 8);
  return val_t;
}

int clean_FSR_values(int address_FSR_l)
{
  for ( unsigned int i = address_FSR_l ; i < (sizeof(double) + sizeof(char) + address_FSR_l) ; i++ )
  {
    EEPROM.write(i, 0);
  }

  return 1;
}


// wirte and read values in EEPROM also per KF, smoothing ,PID and FSR params
// Since it is really big update I would suggest to do it before the trial not in the middle

int write_EXP_parameters(int address_params_l)
{ Serial.println();
  EEPROM.put(address_params_l, 'y');
  address_params_l++;
  EEPROM.put(address_params_l, left_leg->KF);
  address_params_l += 8;
  Serial.println(left_leg->KF);
  EEPROM.put(address_params_l, right_leg->KF);
  address_params_l += 8;
  Serial.println(right_leg->KF);
  EEPROM.put(address_params_l, N1);
  address_params_l += 8;
  Serial.println(N1);
  EEPROM.put(address_params_l, N2);
  address_params_l += 8;
  Serial.println(N2);
  EEPROM.put(address_params_l, N3);
  address_params_l += 8;
  Serial.println(N3);
  EEPROM.put(address_params_l, left_leg->kp);
  address_params_l += 8;
  EEPROM.put(address_params_l, left_leg->kd);
  address_params_l += 8;
  EEPROM.put(address_params_l, left_leg->ki);
  address_params_l += 8;
  EEPROM.put(address_params_l, right_leg->kp);
  address_params_l += 8;
  EEPROM.put(address_params_l, right_leg->kd);
  address_params_l += 8;
  EEPROM.put(address_params_l, right_leg->ki);
  address_params_l += 8;
  EEPROM.put(address_params_l, left_leg->fsr_percent_thresh_Toe);
  address_params_l += 8;
  EEPROM.put(address_params_l, right_leg->fsr_percent_thresh_Toe);
  address_params_l += 8;
  Serial.println("Completed Saving EXP Parameters");

  return 1;
}

int check_EXP_parameters(int address_params_l)
{ Serial.print("Checking at pos ");
  Serial.println(address_params_l);
  byte value = EEPROM.read(address_params_l);
  Serial.println(char(value));
  if (char(value) == 'y') {

    Serial.println(" EXP parameters are present in memory ");
    return 1;
  }
  Serial.println(" EXP parameters are not present in memory ");
  return 0;
}

double read_param(int address_l)
{
  double val_t = 0;
  byte array_values[8];
  byte* p_array = array_values;
  for (unsigned int i = 0; i < sizeof(double); i++)
  {
    *(p_array + i) = EEPROM.read(address_l + i);
  }
  memcpy(&val_t, &array_values, 8);
  return val_t;
}

int read_all_params(int address_params_l) {
  address_params_l++;

  left_leg->KF = read_param(address_params_l);
  address_params_l += 8;
  right_leg->KF = read_param(address_params_l);
  address_params_l += 8;

  N1 = read_param(address_params_l);
  address_params_l += 8;
  N2 = read_param(address_params_l);
  address_params_l += 8;
  N3 = read_param(address_params_l);
  address_params_l += 8;
  left_leg->N1 = N1;
  left_leg->N2 = N2;
  left_leg->N3 = N3;
  right_leg->N1 = N1;
  right_leg->N2 = N2;
  right_leg->N3 = N3;

  left_leg->kp = read_param(address_params_l);
  address_params_l += 8;
  left_leg->kd = read_param(address_params_l);
  address_params_l += 8;
  left_leg->ki = read_param(address_params_l);
  address_params_l += 8;

  right_leg->kp = read_param(address_params_l);
  address_params_l += 8;
  right_leg->kd = read_param(address_params_l);
  address_params_l += 8;
  right_leg->ki = read_param(address_params_l);
  address_params_l += 8;

  left_leg->fsr_percent_thresh_Toe = read_param(address_params_l);
  address_params_l += 8;
  right_leg->fsr_percent_thresh_Toe = read_param(address_params_l);
  address_params_l += 8;

  left_leg->p_steps->fsr_percent_thresh_Toe = left_leg->fsr_percent_thresh_Toe;
  right_leg->p_steps->fsr_percent_thresh_Toe = right_leg->fsr_percent_thresh_Toe;

  return 1;
}

int clean_EXP_Parameters(int address_params_l)
{
  for ( unsigned int i = address_params_l ; i < (sizeof(double) * 13 + sizeof(char) + address_params_l) ; i++ )
  {
    EEPROM.write(i, 0);
  }

  return 1;
}

int write_baseline(int address_baseline_l, double baseline_val)
{
  EEPROM.put(address_baseline_l, 'y');
  address_baseline_l++;
  EEPROM.put(address_baseline_l, baseline_val);
  return 1;
}

double read_baseline(int address_baseline_l)
{
  address_baseline_l += 1;
  double val_t = 0;
  byte array_values[8];
  byte* p_array = array_values;
  for (unsigned int i = 0; i < sizeof(double); i++)
  {
    *(p_array + i) = EEPROM.read(address_baseline_l + i);
  }
  memcpy(&val_t, &array_values, 8);
  return val_t;
}
