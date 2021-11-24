// Functions associated with the auto-adjusting biofeedback system
// Written by: Alyssa Spomer (10-19-21) 

void update_biofeedback_baseline(){ 
  get_biofeedback_baseline(right_leg); 
  get_biofeedback_baseline(left_leg);  
}

void get_biofeedback_baseline(Leg* leg) { // determines whether baseline for the biofeedback system has been run.  
  leg->biofeedback_target_score = 1 + BF_scale;  
  leg->biofeedback_high_val = 0; // need to reset the high value because we now have a new baseline value to normalize by     
  return;
}

void update_biofeedback_high_val(Leg* leg) { // pseudo 'peak finder' algorithm - keeps a record of the highest FSR value for the current step 
  if (leg->state == 3) { 
    if (leg->FSR_Toe_Average/leg->baseline_value > leg->biofeedback_high_val) {  
      leg->biofeedback_high_val = leg->FSR_Toe_Average/leg->baseline_value; // normalized to baseline so that we don't have to display a weird number to the user on the app.  
    }  
  }   
  return; 
}

void refresh_biofeedback(Leg* leg) { // run immediately after a new step is detected. Checks whether or not the previous step was successful and updates the auto-adjusting feature
    if (leg->biofeedback_high_val >= leg->biofeedback_target_score) { // if the individual has hit the target score
      biofeedback_current_success = 1;  // Success = 1
    }
    else {
      biofeedback_current_success = 0; // No success = 0
    }  
    
    // Calculate the success rate of the past 6 strides  
    for (int i = 0; i < leg->biofeedback_window_length; i++){ // cycle through all elements in the array  
      if (leg->biofeedback_success_history[i] == -1) {  
        leg->biofeedback_success_history[i] = biofeedback_current_success;   
        if (i == leg->biofeedback_window_length-1){ // if we are filling the last element in the array, trigger for a new success rate to the calculated 
          leg->FLAG_calc_success_rate = 1; // if we have taken the full number of steps, then the system will adjust itself
        }
        break; 
      }  
      else if (i == leg->biofeedback_window_length - 1){ // if the person has already taken the full number strides (and we haven't modified the target score),remove the first entry and add new entry to the end (to create a sliding 6-step window)  
        for (int k = 0; k < leg->biofeedback_window_length; k++) { 
          if (k < leg->biofeedback_window_length-1) { 
            leg->biofeedback_success_history[k] = leg->biofeedback_success_history[k+1];  
          }
          else { 
            leg->biofeedback_success_history[k] = biofeedback_current_success; 
          }
        } 
      } 
    }   
    // Print the current array   
    /*
    for (int ii = 0; ii < leg->biofeedback_window_length;ii++){  
      Serial.print(leg->biofeedback_success_history[ii]);  
      Serial.print(","); 
    }
    Serial.println("");  
    */
    
    // Calculate the new target score if the autoadjust biofeedback condition is on 
    if (leg->FLAG_calc_success_rate == 1) { // will only calculate a new rate after 6 strides have been taken at the current target goal 
      double success_rate = 0; 
      for (int jj = 0; jj< leg->biofeedback_window_length; jj++){ 
          success_rate += leg->biofeedback_success_history[jj]; 
      }
      leg->biofeedback_success_rate = success_rate/leg->biofeedback_window_length; // calculate the success rate  
      //Serial.println(leg->biofeedback_success_rate);  
  
      // Adjust the target score if the success rate is outside of the set bounds 
      if (leg->biofeedback_success_rate >= BF_upper_limit || leg->biofeedback_success_rate < BF_lower_limit){ 
          if (leg->biofeedback_success_rate >= BF_upper_limit) { // Current threshold for autoadjusting: if the user is taking more than 5 steps at the current treshold, update the score by 10% of baseline
            leg->biofeedback_target_score += BF_scale; 
          }

          if (leg->biofeedback_success_rate <= BF_lower_limit) { // if the user is taking less than 3 steps at the current target score, decrease the score by 10%, but do not decrease below baseline target
            if (leg->biofeedback_target_score-BF_scale >= 1 + BF_scale) { // If the initial baseline threshold is too challenging, keep it there but don't adjust down beyond the original target score, to prevent individuals from manipulating the system
              leg->biofeedback_target_score = leg->biofeedback_target_score-BF_scale;
            }
            else { 
              if (leg->biofeedback_target_score > 1) { 
                leg->biofeedback_target_score = leg->biofeedback_target_score-BF_scale/2; // decrease target score by 0.02 until you reach 1. but don't go below 1 (baseline value) 
              }
            }  
          } 

          //Serial.println("Target Score"); 
          //Serial.println(leg->biofeedback_target_score);    
          for (int j = 0;j<(sizeof(leg->biofeedback_success_history)/sizeof(leg->biofeedback_success_history[1]));j++) { // if the target score is modified, then we reset to ensure that the individual takes atleast 6 steps at each target level! 
            leg->biofeedback_success_history[j] = -1;  
          }
          leg->FLAG_calc_success_rate = 0;  

          // !!! CHANCE - ADD THE CALL TO BLUETOOTH HERE !!!  
      }
    }  





    leg->biofeedback_high_val = 0;  
    return; 
} 

void adjust_biofeedback_success_rate(Leg* leg){ // function to determine if the target score needs to be moved up or down based on the existing success rate 
  
  return; 
}
