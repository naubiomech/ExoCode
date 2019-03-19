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