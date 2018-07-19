void create_data_message(msg *p_msg_1) { 
//  Serial.print(1);
//  Serial.print("\t"); 
//  p_msg_1->msg_array[0]  = 'S';
//  p_msg_1->msg_array[1]  = ',';
//  p_msg_1->msg_array[10] = ',';
//  p_msg_1->msg_array[19] = ',';
//  p_msg_1->msg_array[28] = ',';
//  p_msg_1->msg_array[37] = ',';
//  p_msg_1->msg_array[46] = ',';
//  p_msg_1->msg_array[55] = ',';
//  p_msg_1->msg_array[64] = ',';
//  p_msg_1->msg_array[73] = ',';
//  p_msg_1->msg_array[74] = 'E';
  p_msg_1->msg_array[0]  = 'S';
  p_msg_1->msg_array[1]  = ',';
  p_msg_1->msg_array[6] = ',';
  p_msg_1->msg_array[11] = ',';
  p_msg_1->msg_array[16] = ',';
  p_msg_1->msg_array[21] = ',';
  p_msg_1->msg_array[26] = ',';
  p_msg_1->msg_array[31] = ',';
  p_msg_1->msg_array[36] = ',';
  p_msg_1->msg_array[41] = ',';
  p_msg_1->msg_array[42] = 'E';

  char arr[4] = {0};
  for(int j = 0; j < 8; j++)
  {
    memcpy(arr, (unsigned char*) (&(p_msg_1->N_array[j])), 4);
    for(int i = 0; i < 4; i++)
    {
      p_msg_1->msg_array[2 + j + i + j * 4 ] = arr[i];
    }
  }

//  char arr[8] = {0};
//  for(int j = 0; j < 8; j++)
//  {
//    memcpy(arr, (unsigned char*) (&(p_msg_1->N_array[j])), 8);
//    for(int i = 0; i < 8; i++)
//    {
//      p_msg_1->msg_array[2 + j + i + j * 8 ] = arr[i];
//    }
//  }

  return;
}



//---------------------------------------------------------------------
// Send msg to matlab
void send_message(msg* p_msg_1) {
//  Serial.println(2);
  for (int i = 0; i < 43; i++) {
    if (i == 42) 
    {
      bluetooth.write(p_msg_1 ->msg_array[i]);
    }
    else 
    {
      bluetooth.write(p_msg_1 ->msg_array[i]);
    }
  }

  return;
}
void send_data_message() 
{
  bluetooth.print('S');
  bluetooth.print(',');
  bluetooth.print(T_act_RL);
  bluetooth.print(',');
  bluetooth.print(R_state);
  bluetooth.print(',');
  bluetooth.print(PID_Setpoint_RL);
  bluetooth.print(',');
//  bluetooth.print(fsr(fsr_sense_Right_Toe));
  bluetooth.print(fsr(fsr_sense_Right_Toe));
  bluetooth.print(',');
  bluetooth.print(T_act_LL);
  bluetooth.print(',');
  bluetooth.print(L_state);
  bluetooth.print(',');
  bluetooth.print(PID_Setpoint_LL);
  bluetooth.print(',');
//  bluetooth.print(fsr(fsr_sense_Left_Toe));
  bluetooth.print(fsr(fsr_sense_Left_Toe));
  bluetooth.print(',');
  bluetooth.print(timeElapsed);
  bluetooth.print(',');
  bluetooth.println('Z');
}

void send_command_message(char command_char, double* data_point, int number_to_send) 
{
  bluetooth.print('S');
  bluetooth.print(command_char);
  bluetooth.print(',');
  for(int message_iterator = 0; message_iterator < number_to_send; message_iterator++)
  {
    bluetooth.print(*(data_point + message_iterator));
    bluetooth.print(',');
  }
  bluetooth.println('Z');
}

//---------------------------------------------------------------------
// Receive msg from matlab
/*
int receive_msg(msg* p_msg_l) {

  int k = 0;

  while ((k < 75))
  {
    while (bluetooth.available() > 0)
    {
      int fromBluetooth = bluetooth.read();
      p_msg_l->msg_array[k] = fromBluetooth;               //Store all recieved bytes in subsequent memory spaces
      k = k + 1;                                  //Increments memory space
    }
  }

  p_msg_l->special_character = p_msg_l->msg_array[1];
  
  if ((p_msg_l->msg_array[0] == 'S') && (p_msg_l->msg_array[74] == 'E')) {
    Serial.println("Received correct msg");
    char arr[8];

    for (int j = 0; j < 8; j++) {

      for (int i = 0; i < 8 ; i++) {
        arr[i] = p_msg_l->msg_array[j * (8 + 1) + 2 + i];
      }

      memcpy(p_msg_l->N_array[j], arr, 8);
    }

  } else {
    Serial.println("Not Received correct msg");
    return 0;
  }

  return 1;
}
*/


