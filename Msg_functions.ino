
void send_data_message_wc() //with COP
{
  //if (DEBUG) {Serial.println("In send_data_message_wc()");}
  //Right Leg
  data_to_send[0] = (right_leg->sign * right_leg->Average_Trq);
  data_to_send[1] = right_leg->state;
  data_to_send[2] = (right_leg->sign * right_leg->PID_Setpoint);

  //Left Leg
  data_to_send[3] = (left_leg->sign * left_leg->Average_Trq);
  data_to_send[4] = left_leg->state;
  data_to_send[5] = (left_leg->sign * left_leg->PID_Setpoint);

  send_command_message('?', data_to_send, 6);
 // if (DEBUG) {Serial.println("Out send_data_message_wc()");}
}

void send_command_message(char command_char, double* data_to_send, int number_to_send)
{
  //if (DEBUG) {Serial.println("In send_data_message()");}
  int payloadLength = (3+number_to_send*5);
  byte buffer[payloadLength];
  char c_buffer[4];           
  buffer[0] = 'S';
  buffer[1] = command_char;
  itoa(number_to_send, &c_buffer[0], 10);
  memcpy(&buffer[2],&c_buffer[0],1);
  
  for (int i = 0; i < number_to_send; i++)
  {
    itoa(data_to_send[i]*100, &c_buffer[0], 10);
    memcpy(&buffer[3+i*5], &c_buffer[0], 4);
    buffer[7+i*5] = 'n';  
  }
  TXChar.writeValue(buffer, payloadLength);           //Write payload
  //if (DEBUG) {Serial.println("End send_data_message()");}
}
/*
void send_command_message(char command_char, double* data_to_send, int number_to_send)
{
  if (DEBUG) {Serial.println("In send_data_message()");}
  int size = 10;
  char buffer[size];
  TXChar.writeValue('S');
  TXChar.writeValue(command_char);
  TXChar.writeValue((char) number_to_send + 48);
  float f = 0;
  if (DEBUG) {Serial.println("Start loop");}
  for (int i = 0; i < number_to_send; i++)
  {
    f = data_to_send[i];
    double buffer_for_int = f * 100.0;
    int buff_to_send = (int) buffer_for_int;
    if (DEBUG) {Serial.println("itoa()");}
    itoa(buff_to_send, buffer, size);
    if (DEBUG) {Serial.println("Start nested loop");}
    for (i = 0; i < size; i++)
    {
      TXChar.writeValue(buffer[i]);
    }
    if (DEBUG) {Serial.println("End nested loop");}
    TXChar.writeValue('n');
  }
  if (DEBUG) {Serial.println("End loop");}
  if (DEBUG) {Serial.println("Out send_data_message()");}
}
*/
bool map_expected_bytes() //Determines how much data each command needs before execution
{
  bytesExpected = 0;
  switch (cmd_from_Gui)
  {
    case 'M':
      bytesExpected = 48;
      break;
    case 'F':
    case 'g':
      bytesExpected = 32;
      break;
    case ')':
      bytesExpected = 24;
      break;
    case 'R':
    case '{':
    case '_':
    case '$':
      bytesExpected = 16;
      break;
    case 'X':
    case 'W':
    case 'v':
    case 'a':
    case 'u':
    case '*':
    case '"':
      bytesExpected = 8;
      break;
    case 'f':
      bytesExpected = 1;
      break;
  }
  return ((bytesExpected) ? true : false);  //If zero return false
}
