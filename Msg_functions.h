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

double data_to_send[8];
double *data_to_send_point = &data_to_send[0];

int Sig_1=0;
int Sig_2=0;
int Sig_3=0;
int Sig_4=0;
int Sig_5=0;
int Sig_6=0;
int Sig_7=0;
int Sig_8=0;
int Sig_9=0;
int Sig_10=0;
int Signal_1=0;
int Signal_2=0;
int Signal_3=0;
int Signal_4=0;
