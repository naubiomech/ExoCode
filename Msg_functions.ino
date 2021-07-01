
//Real time data being sent to the GUI
void send_data_message_wc() //with COP
{
  //Right Leg
  data_to_send[0] = (right_leg->sign * right_leg->Average_Trq);
  data_to_send[1] = track_count_R;//right_leg->state;
  data_to_send[2] = (right_leg->sign * right_leg->PID_Setpoint);

  //Left Leg
  data_to_send[3] = (left_leg->sign * left_leg->Average_Trq);
  data_to_send[4] = track_count_L;//left_leg->state;
  data_to_send[5] = (left_leg->sign * left_leg->PID_Setpoint);

  //FSR val
  data_to_send[6] = rate_count_R;//(right_leg->FSR_Toe_Average);
  data_to_send[7] = rate_count_L;//(left_leg->FSR_Toe_Average);

  send_command_message('?', data_to_send, 8);
}

void send_command_message(char command_char, double* data_to_send, int number_to_send)
{
  //6 max characters can transmit -XXXXX, or XXXXXX
  int maxChars = 8;
  int maxPayloadLength = (3 + number_to_send * (maxChars + 1)); //+1 because of the delimiters
  byte buffer[maxPayloadLength];
  //Size must be declared at initialization because of itoa()
  char cBuffer[maxChars];
  int bufferIndex = 0;
  buffer[bufferIndex++] = 'S';
  buffer[bufferIndex++] = command_char;
  itoa(number_to_send, &cBuffer[0], 10);
  memcpy(&buffer[bufferIndex++], &cBuffer[0], 1);
  for (int i = 0; i < number_to_send; i++) {
    //Send as Int to reduce bytes being sent
    int modData = int(data_to_send[i] * 100);
    int cLength = getCharLength(modData);
    //Populates cBuffer with a base 10 number
    itoa(modData, &cBuffer[0], 10);
    //Writes cLength indices of cBuffer into buffer
    memcpy(&buffer[bufferIndex], &cBuffer[0], cLength);
    bufferIndex += cLength;
    buffer[bufferIndex++] = 'n';
  }
  TXChar.writeValue(buffer, bufferIndex);           //Write payload
}

int getCharLength(int ofInt) {
  int len = 0;
  int localInt = ofInt;
  if (localInt < 0) {
    len += 1;
    //Quick abs(x)
    localInt = ((localInt < 0) ? -1 * localInt : localInt);
  }
  //Faster than loop
  if (localInt < 10) {
    len += 1;
  } else if (localInt < 100) {
    len += 2;
  } else if (localInt < 1000) {
    len += 3;
  } else if (localInt < 10000) {
    len += 4;
  } else if (localInt < 100000) {
    len += 5;
  } else if (localInt < 1000000) {
    len += 6;
  }
  return len;
}

bool map_expected_bytes(int& bytesExpected) //Determines how much data each command needs before execution
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

bool handle_matlab_message(char* data, const int data_length) {
  //data is a pointer to a byte buffer
  cmd_from_Gui = *(data+1);
  //-48 to convert from ascii
  int count = (int) (*(data+2));
  count -= 48;
  if (count == 0) {
    //Skip over all the doohickie if you only received a command
    return false;
  }
  //Collect payload
  int i_buff_count{0};
  int i_buff[data_length] = {0};
  int to_collect = count;
  for (int iii = 3; iii < data_length; iii++) {
    if ((*(data+iii)) == 'n') {
      int tmp{0};
      //Epmty buffer
      bool neg = false;
      for (int jjj = 0; jjj < i_buff_count; jjj++) {
        if (i_buff[jjj] < 0) {
          neg = true;
        } else {
          tmp += i_buff[jjj] * pow(10, (i_buff_count - 1) - jjj);
        }
      }
      tmp *= ((neg) ? (-1) : (1));
      double d_tmp = (double)tmp / 100.0;
      memcpy(&holdon[(count - to_collect) * 8], &d_tmp, 8);
      to_collect--;
      i_buff_count = 0;
    }
    else {
      //3 offset because of overhead, -48 to convert from ascii to int
      int tmp = (*(data+iii)) - 48;
      i_buff[i_buff_count++] = tmp;
    }
  }
  return false;
}

bool handle_mobile_message(char* data, const int val_len) {
  static int call{0};
  static int count{0};
  static byte buff[48];
  static int expected{0};
  if (call == 0) {
    cmd_from_Gui = *data;
    call++;
    if (map_expected_bytes(expected)) {
      return true;
    }
    else {
      call = 0;
      return false;
    }
  }
  else { //Call non zero
    //Populate buffer
    memcpy(&buff[count],data,val_len);
    count += val_len;
    if (count == expected) {
      //We have a complete message
      for (int i=0;i<=(expected);i+=8) {
        memcpy(&holdon[i],&buff[i],8);
      }
      call = 0;
      count = 0;
      expected = 0;
      return false;
    }
    else if (count>expected) {
      //Something went wrong, reset
      Serial.println("Error in handle_mobile_message!");
      Serial.print(count);
      Serial.print(expected);
      call = 0;
      count = 0;
      expected = 0;
      return true;
    }
    else {
      return true;
    }
  }
}
