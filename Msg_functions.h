#ifndef MSG_FUNC_H
#define MSG_FUNC_H
// structure to create the messages transmitted and received between arduino and matlab
/* Protocols */
inline void send_thermo_message(double temp);
bool handle_matlab_message(char* data, const int data_length);
bool handle_mobile_message(char* data, const int val_len);

double data_to_send[16];
double *data_to_send_point = &data_to_send[0];


#endif
