//This file will declare some functions to collect data during a trial, and send that data to the gui. 


 struct Trial_Data 
 {
  //# of steps for a given trial
  unsigned int steps = 0;
  //millis() reference for each step, used to determine if step should count
  unsigned long int step_start;
  //Delay to warrant incrementing the step counter
  const int kstep_delay = 250;
  //Trial start time
  unsigned long int trial_start;
};

//Step counter instance
Trial_Data trial_value = Trial_Data();
//Pointer to step counter instance
Trial_Data* stepper = &trial_value;
