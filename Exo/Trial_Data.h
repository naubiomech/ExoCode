//This file will declare some functions to collect data during a trial, and send that data to the gui.


struct Trial_Data
{
  //# of steps for a given trial
  unsigned int steps = 0;
   //Reference steps used for biofeedback tracking
  long int bio_ref_steps = 0;
  //Number of steps performed during biofeedback
  long int bio_steps = 0;
  //millis() reference for each step, used to determine if step should count
  unsigned long int right_step_start;
  unsigned long int left_step_start;
  //Delay to warrant incrementing the step counter
  const int step_delay_k = 250;
  const int step_limit_k = 3500;
  //Trial start time
  unsigned long int trial_start;
  //Int where total steps is stored in EEPROM (See memory map in drive)
  const int kaddr = 183;

  //Flags used by message functions to send steps and time
  const char step_flag = 's';
};

//Step counter instance
Trial_Data trial_value = Trial_Data();
//Pointer to step counter instance
Trial_Data* stepper = &trial_value;
