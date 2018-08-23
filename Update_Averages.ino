//void Update_Averages() {
//
////  for (int j = dim_FSR - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
////  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
////    *(left_leg->p_FSR_Array + j) = *(left_leg->p_FSR_Array + j - 1);                //Puts the element in the following memory space into the current memory space
////    *(right_leg->p_FSR_Array + j) = *(right_leg->p_FSR_Array + j - 1);
////
////    *(left_leg->p_FSR_Array_Heel + j) = *(left_leg->p_FSR_Array_Heel + j - 1);                //Puts the element in the following memory space into the current memory space
////    *(right_leg->p_FSR_Array_Heel + j) = *(right_leg->p_FSR_Array_Heel + j - 1);
////  }
////
////  //Get the FSR
////  *(left_leg->p_FSR_Array) = fsr(left_leg->fsr_sense_Toe);
////  *(right_leg->p_FSR_Array) = fsr(right_leg->fsr_sense_Toe);
////
////  *(left_leg->p_FSR_Array_Heel) = fsr(left_leg->fsr_sense_Heel);
////  *(right_leg->p_FSR_Array_Heel) = fsr(right_leg->fsr_sense_Heel);
//
//
//  //Calc the average value of Torque
//
//  //Shift the arrays
//  for (int j = dim - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
//  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
//    *(left_leg->TarrayPoint + j) = *(left_leg->TarrayPoint + j - 1);                //Puts the element in the following memory space into the current memory space
//    *(right_leg->TarrayPoint + j) = *(right_leg->TarrayPoint + j - 1);
//  }
//
//  //Get the torques
//  *(left_leg->TarrayPoint) = get_LL_torq();
//  *(right_leg->TarrayPoint) = get_RL_torq();
//
//  //  noInterrupts();
//  left_leg->FSR_Average = 0;
//  right_leg->FSR_Average = 0;
//  left_leg->FSR_Average_Heel = 0;
//  right_leg->FSR_Average_Heel = 0;
//  left_leg->Average = 0;
//  right_leg->Average = 0;
//
//  for (int i = 0; i < dim_FSR; i++)
//  {
////    left_leg->FSR_Average = left_leg->FSR_Average + *(left_leg->p_FSR_Array + i);
////    right_leg->FSR_Average = right_leg->FSR_Average + *(right_leg->p_FSR_Array + i);
////
////    left_leg->FSR_Average_Heel = left_leg->FSR_Average_Heel + *(left_leg->p_FSR_Array_Heel + i);
////    right_leg->FSR_Average_Heel = right_leg->FSR_Average_Heel + *(right_leg->p_FSR_Array_Heel + i);
//
//    if (i < dim)
//    {
//      left_leg->Average =  left_leg->Average + *(left_leg->TarrayPoint + i);
//      right_leg->Average =  right_leg->Average + *(right_leg->TarrayPoint + i);
//    }
//  }
//
////  left_leg->Average_Volt = left_leg->FSR_Average / dim_FSR;
////  right_leg->Average_Volt = right_leg->FSR_Average / dim_FSR;
////
////  left_leg->Average_Volt_Heel = left_leg->FSR_Average_Heel / dim_FSR;
////  right_leg->Average_Volt_Heel = right_leg->FSR_Average_Heel / dim_FSR;
//
//  left_leg->Average_Trq = left_leg->Average / dim;
//  right_leg->Average_Trq = right_leg->Average / dim;
//
//  left_leg->p_steps->curr_voltage = (left_leg->FSR_Average + left_leg->FSR_Average_Heel) / dim_FSR;
//  right_leg->p_steps->curr_voltage = (right_leg->FSR_Average + right_leg->FSR_Average_Heel) / dim_FSR;
//  left_leg->p_steps->torque_average = left_leg->Average / dim;
//  right_leg->p_steps->torque_average = right_leg->Average / dim;
//
//}
