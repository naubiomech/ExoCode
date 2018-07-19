  void pid(double input, int Left_or_Right)
  {
    if(Left_or_Right == 1)
    {
     if ((abs(input) > 25))
      {
        bluetooth.println(input);
        bluetooth.println(9);
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(9);//this is fake
        digitalWrite(onoff, LOW);
        stream = 0;
        digitalWrite(13, LOW);
      }
      Input_LL = input;  
      PID_LL.Compute_KF(KF_LL);
      Vol_LL = Output_LL + zero; //need to map
      analogWrite(motor_LL_pin, Vol_LL); //0 to 4096 writing for motor to get Input
    }
    if(Left_or_Right == 2)
    {
     if ((abs(input) > 25))
      {
        bluetooth.println(input);
        bluetooth.println(9);
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(0);//this is fake
        bluetooth.println(9);//this is fake
        digitalWrite(onoff, LOW);
        stream = 0;
        digitalWrite(13, LOW);
      }
     Input_RL = input;  
     PID_RL.Compute_KF(KF_RL);
     Vol_RL = Output_RL + zero; //need to map
     analogWrite(motor_RL_pin, Vol_RL); //0 to 4096 writing for motor to get Input
    }
  }
