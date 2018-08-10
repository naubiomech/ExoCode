#ifndef MSG_FUNCTIONS_HEADER
#define MSG_FUNCTIONS_HEADER

typedef struct {
  char msg_array[75];
  char *p_msg_array = &msg_array[0];
  char start_character = 'S';
  char special_character = ',';
  float N_array[4];
  char end_character = 'E';
  int N_double;
} msg;

void send_data_message_wc(); //with COP
void send_command_message(char command_char, double* data_point, int number_to_send);

#endif
