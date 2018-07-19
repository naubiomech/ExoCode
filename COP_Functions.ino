
double L_COP;
double R_COP;

double L_dist_A_to_H = -0.05; // distance from ankle to heel
double L_dist_A_to_T = 0.1; //distance from ankle to toe

double R_dist_A_to_H = -0.05; // distance from ankle to heel
double R_dist_A_to_T = 0.1; //distance from ankle to toe

double L_COP_Average;
double R_COP_Average;

double L_COP_fun() {

  if (Average_Volt_LL + Average_Volt_LL_Heel < 2)
  {
    return 0;
  }
  else
  {
    return (Average_Volt_LL * L_dist_A_to_T + Average_Volt_LL_Heel * L_dist_A_to_H) / (Average_Volt_LL + Average_Volt_LL_Heel);
  }
}


double R_COP_fun() {

  if (Average_Volt_RL + Average_Volt_RL_Heel < 2)
  {
    return 0;
  }
  else
  {
    return (Average_Volt_RL * R_dist_A_to_T + Average_Volt_RL_Heel * R_dist_A_to_H) / (Average_Volt_RL + Average_Volt_RL_Heel);
  }

}


// In case of Incline
// Strategy 1, if COP evolution remains close to the toe increase dorsiflexion torque
// Strategy 2, if push force inceases, incresase the torque level + Strategy 1
// Stragegy 3, if when you touch and detouch the ground your ankle evolution is different from the plane evolution, 
// as a function of the variation of the ankle angle we can estimate the incline and hence update dorsiflexion.  
//
//double check_incline() {
//  if ((min_L_COP_average >= -.03) && (max_L_COP_average >= 0.8))
//    return 1;
//  else
//    return 0;
//}
