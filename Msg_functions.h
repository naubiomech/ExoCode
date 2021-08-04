// structure to create the messages transmitted and received between arduino and matlab

typedef struct {
  char msg_array[75];
  char *p_msg_array = &msg_array[0];
  char start_character = 'S';
  char special_character = ',';
  float N_array[4];
  char end_character = 'E';
  int N_double;
} msg;

msg val_msg_send;
msg val_msg_receive;
msg* p_msg_send = &val_msg_send;
msg* p_msg_receive = &val_msg_receive;

double data_to_send[16];
double *data_to_send_point = &data_to_send[0];

double emptyData[1]; //for when you need to use send_command_message() but don't need to send actual data
double *emptyData_point = &emptyData[0];

double batteryData[1];
double *batteryData_point = &batteryData[0];

double stepData[2]; //Used to send trial summary information upon end trial command. Note: The size should be changed to three when complete
double *stepData_point = &stepData[0];

double trqCalData[2];
double *trqCalData_point = &trqCalData[0];

double totalSteps[1]; //Used to send the total number of steps
double *totalSteps_point = &totalSteps[0];

double errorCount[1];
