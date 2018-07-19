void Update_Averages() {

  for (int j = dim_FSR - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
    *(p_FSR_Array_LL + j) = *(p_FSR_Array_LL + j - 1);                //Puts the element in the following memory space into the current memory space
    *(p_FSR_Array_RL + j) = *(p_FSR_Array_RL + j - 1);

    *(p_FSR_Array_LL_Heel + j) = *(p_FSR_Array_LL_Heel + j - 1);                //Puts the element in the following memory space into the current memory space
    *(p_FSR_Array_RL_Heel + j) = *(p_FSR_Array_RL_Heel + j - 1);
  }

  //Get the FSR
  //  Curr_FSR_LL = fsr(fsr_sense_Left_Toe);
  //  Curr_FSR_RL = fsr(fsr_sense_Right_Toe);
  *(p_FSR_Array_LL) = fsr(fsr_sense_Left_Toe);
  *(p_FSR_Array_RL) = fsr(fsr_sense_Right_Toe);

  *(p_FSR_Array_LL_Heel) = fsr(fsr_sense_Left_Heel);
  *(p_FSR_Array_RL_Heel) = fsr(fsr_sense_Right_Heel);


  //Calc the average value of Torque

  //Shift the arrays
  for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
    *(TarrayPoint_LL + j) = *(TarrayPoint_LL + j - 1);                //Puts the element in the following memory space into the current memory space
    *(TarrayPoint_RL + j) = *(TarrayPoint_RL + j - 1);
  }

  //Get the torques
  *(TarrayPoint_LL) = get_LL_torq();
  *(TarrayPoint_RL) = get_RL_torq();

  //  noInterrupts();
  FSR_Average_LL = 0;
  FSR_Average_RL = 0;
  FSR_Average_LL_Heel = 0;
  FSR_Average_RL_Heel = 0;
  Average_LL = 0;
  Average_RL = 0;

  for (int i = 0; i < dim_FSR; i++)
  {
    FSR_Average_LL = FSR_Average_LL + *(p_FSR_Array_LL + i);
    FSR_Average_RL = FSR_Average_RL + *(p_FSR_Array_RL + i);

    FSR_Average_LL_Heel = FSR_Average_LL_Heel + *(p_FSR_Array_LL_Heel + i);
    FSR_Average_RL_Heel = FSR_Average_RL_Heel + *(p_FSR_Array_RL_Heel + i);

    if (i < dim)
    {
      Average_LL =  Average_LL + *(TarrayPoint_LL + i);
      Average_RL =  Average_RL + *(TarrayPoint_RL + i);
      //        Average_RL =  Average_RL + *(TarrayPoint_RL + i);
    }
  }

  Average_Volt_LL = FSR_Average_LL / dim_FSR;
  Average_Volt_RL = FSR_Average_RL / dim_FSR;

  Average_Volt_LL_Heel = FSR_Average_LL_Heel / dim_FSR;
  Average_Volt_RL_Heel = FSR_Average_RL_Heel / dim_FSR;

  Average_Trq_LL = Average_LL / dim;
  Average_Trq_RL = Average_RL / dim;

  //  L_p_steps->curr_voltage = FSR_Average_LL / dim_FSR;
  //  R_p_steps->curr_voltage = FSR_Average_RL / dim_FSR;
  L_p_steps->curr_voltage = (FSR_Average_LL + FSR_Average_LL_Heel) / dim_FSR;
  R_p_steps->curr_voltage = (FSR_Average_RL + FSR_Average_RL_Heel) / dim_FSR;
  L_p_steps->torque_average = Average_LL / dim;
  R_p_steps->torque_average = Average_RL / dim;

}
