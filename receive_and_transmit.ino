void receive_and_transmit()
{

  Peek = bluetooth.peek();
  //  Serial.println(Peek);
  if (Peek == 'D')                                                  //if MATLAB sent the character D
  {
    garbage = bluetooth.read();
    bluetooth.println(Setpoint_Ankle_LL);                           //MATLAB is expecting to recieve the Torque Parameters
    Serial.print("Received Left ");
    Serial.println(Setpoint_Ankle_LL);
  }
  if (Peek == 'd')                                                  //if MATLAB sent the character D
  {
    garbage = bluetooth.read();
    bluetooth.println(Setpoint_Ankle_RL);                           //MATLAB is expecting to recieve the Torque Parameters
    Serial.print("Received Right ");
    Serial.println(Setpoint_Ankle_RL);
  }
  if (Peek == 'F')                                                  //If MATLAB sent the character F
  { //MATLAB wants to write a new Torque Value
    garbage = bluetooth.read();
    recieveVals(8);                                                 //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&Setpoint_Ankle_LL, &holdon, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Setpoint_Ankle_LL = abs(Setpoint_Ankle_LL);                     //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
    //Recieved the large data chunk chopped into bytes, a roundabout way was needed
    L_p_steps->Setpoint = Setpoint_Ankle_LL;
    L_activate_in_3_steps = 1;
    L_num_3_steps = 0;
    L_1st_step = 1;
    L_start_step=0;
  }
  if (Peek == 'f')                                                  //If MATLAB sent the character F
  { //MATLAB wants to write a new Torque Value
    garbage = bluetooth.read();
    recieveVals(8);                                                 //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&Setpoint_Ankle_RL, &holdon, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Setpoint_Ankle_RL = -abs(Setpoint_Ankle_RL);                    //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
    //Recieved the large data chunk chopped into bytes, a roundabout way was needed
    R_p_steps->Setpoint = Setpoint_Ankle_RL;
    R_activate_in_3_steps = 1;
    R_num_3_steps = 0;
    R_1st_step = 1;
    R_start_step=0;
  }
  if (Peek  == 'E')                                                 //If MATLAB sent the character E
  {
    garbage = bluetooth.read();
    digitalWrite(22, HIGH);                                         //The GUI user is ready to start the trial so Motor is enabled
    // the bluetooth provide 8 values
    bluetooth.println(0);
    bluetooth.println(0);
    bluetooth.println(0);
    bluetooth.println(0);
    bluetooth.println(0);
    bluetooth.println(0);                                           // this has been added to plot also the set point
    bluetooth.println(0);
    bluetooth.println(0);
    stream = 1;                                                     //and the torque data is allowed to be streamed
    streamTimerCount = 0;
  }
  if (Peek == 'G')                                                 //If MATLAB sent the character G
  {
    garbage = bluetooth.read();
    digitalWrite(22, LOW);                                         //The GUI user is ready to end the trial, so motor is disabled
    stream = 0;                                                    //and the torque data is no longer allowed to be streamed.
    //*Note* Even though no new data is being streamed the buffer in MATLAB still may contain data that has not been read in yet
    //Or data is still "In transit" that needs to be accounted for on the MATLAB side or the GUI will start doing Wierd things
  }
  if (Peek == 'H')                                                 //If MATLAB sent the character H
  {
    garbage = bluetooth.read();
    //digitalWrite(13, LOW);                                         //The user Wants to Recalibrate all the Sensors
    torque_calibration();
    write_torque_bias(address_torque_LL, Tcal_LL);
    write_torque_bias(address_torque_RL, Tcal_RL);

  }
  if (Peek == 'K')                                                //If MATLAB sent the character K, then it wants to know what the current PID Parameters are
  {
    garbage = bluetooth.read();
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    bluetooth.println(kp_LL);                                      //Sends the PID parameters
    bluetooth.println(kd_LL);
    bluetooth.println(ki_LL);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }
  if (Peek == 'k')                                                //If MATLAB sent the character K, then it wants to know what the current PID Parameters are
  {
    garbage = bluetooth.read();
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    bluetooth.println(kp_RL);                                      //Sends the PID parameters
    bluetooth.println(kd_RL);
    bluetooth.println(ki_RL);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }
  if (Peek == 'L')
  {
    garbage = bluetooth.read();

    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;

    FSR_calibration();
    //double fsr_Left_Heel_thresh;
    //double fsr_Left_Toe_thresh;
    //double fsr_Right_Heel_thresh;
    //double fsr_Right_Toe_thresh;
    write_FSR_values(address_FSR_LL, fsr_Left_Heel_thresh);
    write_FSR_values((address_FSR_LL + sizeof(double) + sizeof(char)), fsr_Left_Toe_thresh);
    write_FSR_values(address_FSR_RL, fsr_Right_Heel_thresh);
    write_FSR_values((address_FSR_RL + sizeof(double) + sizeof(char)), fsr_Right_Toe_thresh);

    L_p_steps->voltage_ref = fsr_Left_Toe_thresh;
    R_p_steps->voltage_ref = fsr_Right_Toe_thresh;

    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;

  }

  if (Peek == 'M')                                                  //If MATLAB Sent the character M, then it wants to write new PID parameters
  {
    garbage = bluetooth.read();
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    Serial.println('M');
    recieveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
    //MATLAB Sent Kp, then Kd, then Ki.
    memcpy(&kp_LL, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    memcpy(&kd_LL, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
    memcpy(&ki_LL, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
    PID_LL.SetTunings(kp_LL, ki_LL, kd_LL);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }
  if (Peek == 'm')                                                  //If MATLAB Sent the character M, then it wants to write new PID parameters
  {
    garbage = bluetooth.read();
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    recieveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
    //MATLAB Sent Kp, then Kd, then Ki.
    memcpy(&kp_RL, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    memcpy(&kd_RL, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_R.  Essentially a roundabout way to change a variable value, but since the bluetooth
    memcpy(&ki_RL, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
    PID_RL.SetTunings(kp_RL, ki_RL, kd_RL);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }

  if (Peek == 'N')                                                  //If MATLAB sent the character N, it wants to check that the Arduino is behaving
  {
    garbage = bluetooth.read();
    //digitalWrite(13,LOW);
    bluetooth.println("B");                                         //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
  }
  if (Peek == '<')        //Check the EEPROM
  {
    garbage = bluetooth.read();
    if ((check_torque_bias(address_torque_LL)) && (check_torque_bias(address_torque_RL)))
    {
      Tcal_LL = read_torque_bias(address_torque_LL);
      Tcal_RL = read_torque_bias(address_torque_RL);
      Tarray_LL[3] = {0};
      Tarray_RL[3] = {0};
      bluetooth.println('y');
    }
    else
    {
      bluetooth.println('*');
    }

    //    if (check_torque_bias(address_torque_RL))
    //    {
    //      Tcal_RL = read_torque_bias(address_torque_RL);
    //      bluetooth.println('y');
    //    }
    //    else
    //    {
    //      bluetooth.println('*');
    //    }

    if (((check_FSR_values(address_FSR_LL)) && (check_FSR_values(address_FSR_LL + sizeof(double) + 1))) &&
        ((check_FSR_values(address_FSR_RL)) && (check_FSR_values(address_FSR_RL + sizeof(double) + 1))))
    {
      fsr_Left_Heel_thresh = read_FSR_values(address_FSR_LL);
      fsr_Left_Toe_thresh = read_FSR_values(address_FSR_LL + sizeof(double) + 1);
      fsr_Right_Heel_thresh = read_FSR_values(address_FSR_RL);
      fsr_Right_Toe_thresh = read_FSR_values(address_FSR_RL + sizeof(double) + 1);
      bluetooth.println('y');
      Serial.print("Left values: ");
      Serial.print(fsr_Left_Toe_thresh);
      Serial.print(", ");
      Serial.print("Right values: ");
      Serial.print(fsr_Right_Toe_thresh);


    }
    else
    {
      bluetooth.println('*');
    }

    //    if ((check_FSR_values(address_FSR_RL)) && (check_FSR_values(address_FSR_RL + sizeof(double) + 1)))
    //    {
    //      fsr_Right_Heel_thresh = read_FSR_values(address_FSR_RL);
    //      fsr_Right_Toe_thresh = read_FSR_values(address_FSR_RL + sizeof(double) + 1);
    //      bluetooth.println('y');
    //    }
    //    else
    //    {
    //      bluetooth.println('*');
    //    }

    L_p_steps->voltage_ref = fsr_Left_Toe_thresh;
    R_p_steps->voltage_ref = fsr_Right_Toe_thresh;
  }// end if Check Memory
  if (Peek == '>')
  { // Clean Memory
    garbage = bluetooth.read();
    //------------------------------------------
    if (clean_torque_bias(address_torque_LL))
    {
      Serial.println("Clear Torque ");
    }
    else
    {
      Serial.println("No clear Torque");
    }
    if (clean_FSR_values(address_FSR_LL))
    {
      Serial.println("Clear FSR ");
    }
    else
    {
      Serial.println("No clear FSR");
    }
    //------------------------------------------
    if (clean_torque_bias(address_torque_RL))
    {
      Serial.println("Clear Torque ");
    }
    else
    {
      Serial.println("No clear Torque");
    }
    if (clean_FSR_values(address_FSR_RL))
    {
      Serial.println("Clear FSR ");
    }
    else
    {
      Serial.println("No clear FSR");
    }
    //------------------------------------------
    if (clean_FSR_values(address_FSR_LL + sizeof(double) + sizeof(char)))
    {
      Serial.println("Clear FSR ");
    }
    else
    {
      Serial.println("No clear FSR");
    }
    if (clean_FSR_values(address_FSR_RL + sizeof(double) + sizeof(char)))
    {
      Serial.println("Clear FSR ");
    }
    else
    {
      Serial.println("No clear FSR");
    }
  }// end if clean memory

  if (Peek == '_')
  { //Matlab wants to set KF_LL
    garbage = bluetooth.read();
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    recieveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&KF_LL, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    KF_RL = store_KF_RL;
  }
  if (Peek == '-')
  { //MATLAB wants to set KF_RL
    garbage = bluetooth.read();
    double store_KF_LL = KF_LL;
    KF_LL = 0;
    KF_RL = 0;
    recieveVals(8);
    memcpy(&KF_RL, &holdon, 8);
    KF_LL = store_KF_LL;
  }
  if (Peek == '`')
  { //Matlab wants to check KF_LL
    garbage = bluetooth.read();
    bluetooth.println(KF_LL);
  }
  if (Peek == '~')
  { //Matlab wants to check KF_RL
    garbage = bluetooth.read();
    bluetooth.println(KF_RL);
  }
  if (Peek == ')')
  {
    garbage = bluetooth.read();
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    recieveVals(24);                               //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
    //MATLAB Sent N1 N2 and then N3 Paramenters for smoothing (see change pid setpoint)
    memcpy(&N1, holdOnPoint, 8);                   //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    memcpy(&N2, holdOnPoint + 8, 8);               //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
    memcpy(&N3, holdOnPoint + 16, 8);              //Recieved the large data chunk chopped into bytes, a roundabout way was needed

    N1_RL = N1;
    N2_RL = N2;
    N3_RL = N3;

    N1_LL = N1;
    N2_LL = N2;
    N3_LL = N3;

    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
    Serial.print("Set Smooth ");
    Serial.print(" ");
    Serial.print(N1);
    Serial.print(" ");
    Serial.print(N2);
    Serial.print(" ");
    Serial.print(N3);
    Serial.println();
  }

  if (Peek == '(')                                 //If MATLAB Sent the character M, then it wants to write new PID parameters
  {
    garbage = bluetooth.read();
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    bluetooth.println(N1);                //N1, N2 and N3 are currently not defined, I am temporarily commenting them out to see if my changes still compile
    bluetooth.println(N2);
    bluetooth.println(N3);

    Serial.print("Get Smooth ");
    Serial.print("");
    Serial.print(N1);
    Serial.print("");
    Serial.print(N2);
    Serial.print("");
    Serial.print(N3);
    Serial.println();

    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }

  if (Peek == 'P')
  {
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    L_p_steps->flag_take_baseline = true;
    Serial.println("Left Freq Baseline ");
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }

  if (Peek == 'p')
  {
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    R_p_steps->flag_take_baseline = true;
    Serial.println("Right Freq Baseline ");
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }

  if (Peek == 'O')
  {
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    L_p_steps->flag_N3_adjustment_time = true;
    Serial.println(" Left N3 Adj ");
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }


  if (Peek == 'o')
  {
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    R_p_steps->flag_N3_adjustment_time = true;
    Serial.println(" Right N3 Adj ");
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }

  if (Peek == 'Q')
  { //Matlab wants to check FSR
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    bluetooth.println(fsr_percent_thresh_Left_Toe);
    Serial.print("Checking the fsr_percent_thresh_Left_Toe: ");
    Serial.println(fsr_percent_thresh_Left_Toe);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }//end if set KF

  if (Peek == 'q')
  { //Matlab wants to check FSR
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    bluetooth.println(fsr_percent_thresh_Right_Toe);
    Serial.print("Checking the fsr_percent_thresh_Right_Toe: ");
    Serial.println(fsr_percent_thresh_Right_Toe);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }//end if set KF

  if (Peek == 'R')
  { //Matlab wants to set FSR
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    delay(10);
    recieveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&fsr_percent_thresh_Left_Toe, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Serial.print("Setting the fsr_percent_thresh_Left_Toe: ");
    Serial.println(fsr_percent_thresh_Left_Toe);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }//end if

  if (Peek == 'r')
  { //Matlab wants to set FSR
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    delay(10);
    recieveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&fsr_percent_thresh_Right_Toe, &holdon, 8);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;//Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Serial.print("Setting the fsr_percent_thresh_Rigth_Toe: ");
    Serial.println(fsr_percent_thresh_Right_Toe);

  }//end if


  if (Peek == 'S')
  { //Matlab wants to set FSR
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    delay(10);
    recieveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&(L_p_steps->perc_l), &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Serial.print("Setting the L_p_steps->perc_l: ");
    Serial.println(L_p_steps->perc_l);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }//end if

  if (Peek == 's')
  { //Matlab wants to set FSR
    double store_KF_LL = KF_LL;
    double store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;
    garbage = bluetooth.read();
    delay(10);
    recieveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&(R_p_steps->perc_l), &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Serial.print("Setting the R_p_steps->perc_l: ");
    Serial.println(R_p_steps->perc_l);
    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
  }//end if
  
  if (Peek == 'C'){ // Clear the buffer
while(bluetooth.available()>0) bluetooth.read(); 
Serial.println("Buffer Clean");
  }

  Peek = 0;
}


