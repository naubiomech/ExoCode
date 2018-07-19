//void FSR_Average_function(FSR_average_struct* p_FSR_l, steps* L_p_steps_l, steps* R_p_steps_l) {
//
//  for (int j = dim_FSR; j >= 0; j--)                    //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
//  { // there are the number of spaces in the memory space minus 2 actions that need to be taken
//    *(p_FSR_l->p_FSR_Array_LL + j) = *(p_FSR_l->p_FSR_Array_LL + j - 1);                //Puts the element in the following memory space into the current memory space
//    *(p_FSR_l->p_FSR_Array_RL + j) = *(p_FSR_l->p_FSR_Array_RL + j - 1);
//  }
//
//  //Get the FSR
//  p_FSR_l->Curr_FSR_LL = fsr(fsr_sense_Left_Toe);
//  p_FSR_l->Curr_FSR_RL = fsr(fsr_sense_Right_Toe);
//  *(p_FSR_l->p_FSR_Array_LL) = p_FSR_l->Curr_FSR_LL;
//  *(p_FSR_l->p_FSR_Array_RL) = p_FSR_l->Curr_FSR_RL;
//
//  p_FSR_l->FSR_Average_LL = 0;
//  p_FSR_l->FSR_Average_RL = 0;
//  for (int i = 0; i < dim_FSR; i++)
//  {
//    p_FSR_l->FSR_Average_LL = p_FSR_l->FSR_Average_LL + *(p_FSR_l->p_FSR_Array_LL + i);
//    p_FSR_l->FSR_Average_RL = p_FSR_l->FSR_Average_RL + *(p_FSR_l->p_FSR_Array_RL + i);
//  }
//  p_FSR_l->FSR_Average_LL = p_FSR_l->FSR_Average_LL / dim_FSR;
//  p_FSR_l->FSR_Average_RL = p_FSR_l->FSR_Average_RL / dim_FSR;
//
//  L_p_steps_l->curr_voltage = p_FSR_l->FSR_Average_LL;
//  R_p_steps_l->curr_voltage = p_FSR_l->FSR_Average_RL;
//
//  return;
//}
