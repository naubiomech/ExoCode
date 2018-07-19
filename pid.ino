void pid(double input) {

  if ((abs(input) > 25))
  {
    bluetooth.println(input);
    bluetooth.println(9);


    digitalWrite(onoff, LOW);
    stream = 0;
    digitalWrite(13, LOW);
  }

  Input_LL = input;
  PID_LL.Compute();
  Vol_LL = Output_LL + zero; //need to map
  analogWrite(A14, Vol_LL); //0 to 4096 writing for motor to get Input
}
