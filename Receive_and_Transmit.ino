// cmd_from_Gui is the variable used to identify the message received by matlab


void receive_and_transmit()
{
  switch (cmd_from_Gui)
  {
    case 'E':
      stream = true;
      break;

    case 'G':
      stream = false;
      break;
  }
  cmd_from_Gui = 0;
}
