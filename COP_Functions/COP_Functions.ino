
double L_COP;
double R_COP;

double L_dist_A_to_H =-0.05; // distance from ankle to heel
double L_dist_A_to_T = 0.1; //distance from ankle to toe

double R_dist_A_to_H =-0.05; // distance from ankle to heel
double R_dist_A_to_T = 0.1; //distance from ankle to toe

double L_COP_fun() {
  
  return (fsr(fsr_sense_Left_Toe)*L_dist_A_to_T+fsr(fsr_sense_Left_Heel)*L_dist_A_to_H)/(fsr(fsr_sense_Left_Toe)+fsr(fsr_sense_Left_Heel));
//  Curr_Left_Heel = fsr(fsr_sense_Left_Heel);
//  Curr_Right_Toe = fsr(fsr_sense_Right_Toe);
//  Curr_Right_Heel = fsr(fsr_sense_Right_Heel);

}

double R_COP_fun() {
  
//  Curr_Left_Toe = fsr(fsr_sense_Left_Toe);
//  Curr_Left_Heel = fsr(fsr_sense_Left_Heel);
//  Curr_Right_Toe = fsr(fsr_sense_Right_Toe);
//  Curr_Right_Heel = fsr(fsr_sense_Right_Heel);

}
