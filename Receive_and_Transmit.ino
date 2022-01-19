// cmd_from_Gui is the variable used to identify the message received by matlab

double app = 0;

void receive_and_transmit()
{
  switch (cmd_from_Gui)
  {
    case 'F':                                                 //MATLAB is only sending 1 value, a double, which is 8 bytes
      //Recieve torque commands
      left_leg->Previous_Setpoint_Ankle = left_leg->Setpoint_Ankle;
      left_leg->Previous_Dorsi_Setpoint_Ankle = left_leg->Dorsi_Setpoint_Ankle;
      right_leg->Previous_Setpoint_Ankle = right_leg->Setpoint_Ankle;
      right_leg->Previous_Dorsi_Setpoint_Ankle = right_leg->Dorsi_Setpoint_Ankle;
      memcpy(&left_leg->Setpoint_Ankle, holdOnPoint, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&left_leg->Dorsi_Setpoint_Ankle, holdOnPoint + 8, 8);
      memcpy(&right_leg->Setpoint_Ankle, holdOnPoint + 16, 8);                        //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&right_leg->Dorsi_Setpoint_Ankle, holdOnPoint + 24, 8);
//      Serial.println(left_leg->Setpoint_Ankle);
//      Serial.println(left_leg->Dorsi_Setpoint_Ankle);
//      Serial.println(right_leg->Setpoint_Ankle);
//      Serial.println(right_leg->Dorsi_Setpoint_Ankle);

      if (left_leg->Setpoint_Ankle < 0) {
        left_leg->Setpoint_Ankle = 0;
        left_leg->Previous_Setpoint_Ankle = 0;
        left_leg->Dorsi_Setpoint_Ankle = 0;
        left_leg->Previous_Dorsi_Setpoint_Ankle = 0;
        left_leg->coef_in_3_steps = 0;
        left_leg->activate_in_3_steps = 1;
        left_leg->first_step = 1;
        left_leg->num_3_steps = 0;
        left_leg->start_step = 0;
      } else {
        left_leg->Setpoint_Ankle = abs(left_leg->Setpoint_Ankle);                     //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
        left_leg->Dorsi_Setpoint_Ankle = -abs(left_leg->Dorsi_Setpoint_Ankle);
        //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        left_leg->Previous_Setpoint_Ankle_Pctrl = left_leg->Previous_Setpoint_Ankle;  // TN 7/25/19
        left_leg->activate_in_3_steps = 1;
        left_leg->num_3_steps = 0;
        left_leg->first_step = 1;
        left_leg->start_step = 0;
      }

      // TN 6/13/19
      if (right_leg->Setpoint_Ankle < 0) {

        right_leg->Setpoint_Ankle = 0;
        right_leg->Dorsi_Setpoint_Ankle = 0;
        right_leg->Previous_Setpoint_Ankle = 0;
        right_leg->Previous_Dorsi_Setpoint_Ankle = 0;
        right_leg->coef_in_3_steps = 0;
        right_leg->activate_in_3_steps = 1;
        right_leg->first_step = 1;
        right_leg->num_3_steps = 0;
        right_leg->start_step = 0;

      } else {

        right_leg->Setpoint_Ankle = -abs(right_leg->Setpoint_Ankle);                    //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
        right_leg->Dorsi_Setpoint_Ankle = abs(right_leg->Dorsi_Setpoint_Ankle);
        //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        right_leg->Previous_Setpoint_Ankle_Pctrl = right_leg->Previous_Setpoint_Ankle;
        right_leg->activate_in_3_steps = 1;
        right_leg->num_3_steps = 0;
        right_leg->first_step = 1;
        right_leg->start_step = 0;
      }

      break;

    case 'E':
      change_motor_state(true);                                      //The GUI user is ready to start the trial so Motor is enabled
      stream = 1;                                                     //and the torque data is allowed to be streamed
      streamTimerCount = 0;
      timeElapsed = 0;
      stepper->trial_start = millis();  //CFC 1/22/21 Gathers time at start of trial
      /* Send torque calibration values */
      trqCalData[0] = right_leg->torque_calibration_value;
      trqCalData[1] = left_leg->torque_calibration_value;
      send_command_message('H', trqCalData, 2);
      break;

    case 'G':
      left_leg->p_steps->Setpoint = 0.0;
      left_leg->Setpoint_Ankle = 0;
      left_leg->Previous_Setpoint_Ankle = 0;
      left_leg->Setpoint_Ankle_Pctrl = 0;
      left_leg->Previous_Setpoint_Ankle_Pctrl = 0;
      left_leg->Dorsi_Setpoint_Ankle = 0;
      left_leg->Previous_Dorsi_Setpoint_Ankle = 0;
      left_leg->coef_in_3_steps = 0;
      left_leg->activate_in_3_steps = 1;
      left_leg->first_step = 1;
      left_leg->num_3_steps = 0;
      left_leg->start_step = 0;
      left_leg->baseline_value = 0;
      left_leg->PID_Setpoint = 0.0; //Makes sure that the next trial doesn't have any non-zero bias
      left_leg->New_PID_Setpoint = 0.0;
      left_leg->Old_PID_Setpoint = 0.0;
      left_leg->KF = 1.0;

      // FSR Biofeedback - AS 11/16/21 
      left_leg->biofeedback_high_val = 0; 
      left_leg->FLAG_calc_success_rate = 0; 

      right_leg->p_steps->Setpoint = 0.0;
      right_leg->Setpoint_Ankle = 0;
      right_leg->Dorsi_Setpoint_Ankle = 0;
      right_leg->Setpoint_Ankle_Pctrl = 0;
      right_leg->Previous_Setpoint_Ankle_Pctrl = 0;
      right_leg->Previous_Setpoint_Ankle = 0;
      right_leg->Previous_Dorsi_Setpoint_Ankle = 0;
      right_leg->coef_in_3_steps = 0;
      right_leg->activate_in_3_steps = 1;
      right_leg->first_step = 1;
      right_leg->num_3_steps = 0;
      right_leg->start_step = 0;
      right_leg->baseline_value = 0;
      right_leg->PID_Setpoint = 0;
      right_leg->New_PID_Setpoint = 0;
      right_leg->start_step = 0;   
      right_leg->PID_Setpoint = 0.0;
      right_leg->New_PID_Setpoint = 0.0;
      right_leg->Old_PID_Setpoint = 0.0;
      right_leg->KF = 1.0; 

      //FSR Biofeedback - AS 11/16/21 
      right_leg->biofeedback_high_val = 0; 
      right_leg->FLAG_calc_success_rate = 0; 
      for (int j = 0;j<(sizeof(right_leg->biofeedback_success_history)/sizeof(right_leg->biofeedback_success_history[1]));j++) {
        right_leg->biofeedback_success_history[j] = -1; 
        left_leg->biofeedback_success_history[j] = -1; 
      }

      reset_count = 0;

      change_motor_state(false);                                        //The GUI user is ready to end the trial, so motor is disabled
      stepData[0] = stepper->steps; //CFC 1/22/21
      stepData[1] = (stepper->bio_steps) / 2; //Divide by two because only one leg is being used during biofeedback
      send_command_message(stepper->step_flag, stepData, 2);
      stream = 0;                                                    //and the torque data is no longer allowed to be streamed.
      stepper->steps = 0;                                            // Reset step count for next trial
      stepper->bio_steps = 0;
      
      reset_cal_on_end_trial(); //Currently commented out but acts to reset all torque, BL, and FSR parameters
      break;

    case 'H':
      //Calibrate torque sensors
      torque_calibration();
      break;

    case 'K':
      //Send PID gains
      *(data_to_send_point) = left_leg->kp;
      *(data_to_send_point + 1) = left_leg->kd;
      *(data_to_send_point + 2) = left_leg->ki;
      *(data_to_send_point + 3) = right_leg->kp;
      *(data_to_send_point + 4) = right_leg->kd;
      *(data_to_send_point + 5) = right_leg->ki;
      send_command_message('K', data_to_send_point, 6);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'k':
//      CURRENT_CONTROL = !CURRENT_CONTROL; //GO 12/4/2019 - Enable/Disable open-loop current control based on GUI checkbox
//      CURRENT_DIAGNOSTICS = 0;
//      MODEL_CONTROL = 0;
      break;

    case 'L':
      //Calibrate FSR command
      FSR_CAL_FLAG = 1;

      break;

    case 'N':
      //Increment Mark Count
      markCount++;
      markFlag = true;
      break;

    case 'S':
      Control_Mode = 6;
      break;
      
    case 'c':
      Control_Mode = 4; // TN 04/29/19
      break;

    case 'l': // TN 04/29/19
//      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR; // TN 04/29/19
//      FLAG_ONE_TOE_SENSOR = true; // TN 04/29/19
//      Old_Control_Mode = Control_Mode; // TN 04/29/19
//      Flag_Prop_Ctrl = true; // TN 04/29/19
//      if (flag_pivot == true) {   // TN 04/29/19
//        Control_Mode = 3; // activate pivot PC // TN 04/29/19
//      } else if (flag_id == true) { // TN 04/29/19
//        Control_Mode = 4; // activate ID PC // TN 04/29/19
//      } else if (flag_resist == true) {
//        Control_Mode = 6; // Activate resistance control //GO 6/20/2020
//      }
//      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint; // TN 04/29/19
//      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint; // TN 04/29/19
      break;

    case 'w':
      //Motors off command
      change_motor_state(false);
      break;

    case 'x':
      //Motors on command
      change_motor_state(true);
      break;

    case '%':
      //Start Biofeedback
      stepper->bio_ref_steps = stepper->steps; 
      FLAG_BIOFEEDBACK = true; // added by AS - 11/8/21    
      right_leg->biofeedback_first_step = true;  
      left_leg->biofeedback_first_step = true; 
      // Serial.println(FLAG_BIOFEEDBACK); 
      //Serial.println(stepper->bio_ref_steps);
      break;

    case 'h':
      //End Biofeedback
      stepper->bio_steps += stepper->steps - stepper->bio_ref_steps; 
      FLAG_BIOFEEDBACK = false; // added by AS - 11/8/21   
      AUTOADJUST_BIOFEEDBACK = false;
      //Serial.println(stepper->bio_steps);
      break;
      
    case 'd': 
      // Toggle autoadjusting biofeedback 
      AUTOADJUST_BIOFEEDBACK = !AUTOADJUST_BIOFEEDBACK;
      break;  

    case 'q': 
      //Change the leg for dynamic biofeedback
      char temp;
      memcpy(&temp, &holdOnPoint[0], 8);
      biofeedbackLeg = (temp == '0' ? 'L':'R');
//      Serial.print("Changing leg to: ");
//      Serial.println(biofeedbackLeg);
      break; 

    // TN 6/13/19
    case 'M':                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent Kp, then Kd, then Ki.
      memcpy(&left_leg->kp, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&left_leg->kd, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&left_leg->ki, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
      memcpy(&right_leg->kp, holdOnPoint + 24, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&right_leg->kd, holdOnPoint + 32, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&right_leg->ki, holdOnPoint + 40, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
      left_leg->pid.SetTunings(left_leg->kp, left_leg->ki, left_leg->kd);
      right_leg->pid.SetTunings(right_leg->kp, right_leg->ki, right_leg->kd);
      break;

    case 'f':
//      memcpy(&MotorParams, holdOnPoint, 1); //Copy the value sent from MATLAB into the MotorParams variable to define motor parameters
//      if (MotorParams == 0) {
//        // 22mm 90W motor, 22HP gearbox
//
//        MaxSpeed = 15000; //RPM
//        TrqConstant = 14 / 1000; //Nm/A
//        GearRatio = 4617.0 / 52.0; //89:1 gear ratio
//        NomCurrent = 7.58; //A
//        MotorEff = 0.89;
//        GearboxEff = 0.59;
//        PulleyRatio = 40 / 10.3; //Small aluminum pulley, large sprocket
//        Serial.println("22mm 90W");
//
//      } else if (MotorParams == 1) {
//        // 22mm 120W motor, 32HP C gearbox
//
//        MaxSpeed = 15800; //RPM
//        TrqConstant = 13.5 / 1000; //Nm/A
//        GearRatio = 6877.0 / 56.0; //123:1 gear ratio
//        NomCurrent = 7.58; //A
//        MotorEff = 0.89;
//        GearboxEff = 0.7;
//        PulleyRatio = 30 / 10.3; //Small aluminum pulley, large sprocket
//        Serial.println("22mm 120W");
//
//      } else if (MotorParams == 2) {
//        // 30mm 200W motor, 32HP gearbox (51:1)
//
//        MaxSpeed = 16100; //RPM
//        TrqConstant = 13.6 / 1000; //Nm/A
//        GearRatio = 17576.0 / 343.0; //51:1 gear ratio
//        NomCurrent = 7.58; //A
//        MotorEff = 0.89;
//        GearboxEff = 0.7;
//        PulleyRatio = 74 / 10.3; //Large aluminum pulley, large sprocket
//        Serial.println("30mm 200W 51:1");
//
//      } else if (MotorParams == 3) {
//        // 30mm 200W motor, 32HP gearbox (103:1)
//
//        MaxSpeed = 16100; //RPM
//        TrqConstant = 13.6 / 1000; //Nm/A
//        GearRatio = 3588.0 / 35.0; //103:1 gear ratio
//        NomCurrent = 7.58; //A
//        MotorEff = 0.89;
//        GearboxEff = 0.7;
//        PulleyRatio = 30 / 13.25; //Carbon fiber pulley, motor pulley
//        Serial.println("30mm 200W 103:1");
//
//      }

      break;

    // TN 6/13/19
    case '_':                                          //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->KF, holdOnPoint, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&right_leg->KF, holdOnPoint + 8, 8);
      break;


    case'`':  // TN 7/3/19
      *(data_to_send_point) = left_leg->KF;
      *(data_to_send_point + 1) = right_leg->KF;
      send_command_message('`', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      break;


    case')':                            //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent N1 N2 and then N3 Paramenters for smoothing (see change pid setpoint)
      memcpy(&N1, holdOnPoint, 8);                   //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&N2, holdOnPoint + 8, 8);               //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&N3, holdOnPoint + 16, 8);              //Recieved the large data chunk chopped into bytes, a roundabout way was needed

      right_leg->N1 = N1;
      right_leg->N2 = N2;
      right_leg->N3 = N3;

      left_leg->N1 = N1;
      left_leg->N2 = N2;
      left_leg->N3 = N3;

      left_leg->old_N3 = left_leg->N3;
      left_leg->old_N2 = left_leg->N2;
      left_leg->old_N1 = left_leg->N1;

      right_leg->old_N3 = right_leg->N3;
      right_leg->old_N2 = right_leg->N2;
      right_leg->old_N1 = right_leg->N1;

      break;

    case '(':
      *(data_to_send_point) = N1;
      *(data_to_send_point + 1) = N2;
      *(data_to_send_point + 2) = N3;
      send_command_message('(', data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters

      break;

//    case 'O': // SS 8/6/2020
//      if (Trigger_left) {
//        left_leg->trig_number = 1;
//        left_leg->trig_time = millis();
//        left_leg->Approve_trigger = false;
//        left_leg->trig1_counter += 1;
//        *(data_to_send_point) = left_leg->trig1_counter;
//      } else {
//        right_leg->trig_number = 1;
//        right_leg->trig_time = millis();
//        right_leg->Approve_trigger = false;
//        right_leg->trig1_counter += 1;
//        *(data_to_send_point) = right_leg->trig1_counter;
//      }
//      send_command_message('O', data_to_send_point, 1);
//      break;
//
//    case 'd':// SS 8/6/2020
//      STIM_ACTIVATED = true;
//      break;
//
//    case'~':// SS 8/6/2020
//      STIM_ACTIVATED = false;
//      break;
//
//    case 'q':// SS 8/6/2020
//      if (Trigger_left) {
//        left_leg->trig_number = 2;
//        left_leg->trig_time = millis();
//        left_leg->Approve_trigger = false;
//        left_leg->trig2_counter += 1;
//        *(data_to_send_point) = left_leg->trig2_counter;
//      } else {
//        right_leg->trig_number = 2;
//        right_leg->trig_time = millis();
//        right_leg->Approve_trigger = false;
//        right_leg->trig2_counter += 1;
//        *(data_to_send_point) = right_leg->trig2_counter;
//      }
//      send_command_message('q', data_to_send_point, 1);
//      break;
//
//    case '[':// SS 8/6/2020
//      if (Trigger_left) {
//        left_leg->trig_number = 3;
//        left_leg->trig_time = millis();
//        left_leg->Approve_trigger = false;
//        left_leg->trig3_counter += 1;
//        *(data_to_send_point) = left_leg->trig3_counter;
//      } else {
//        right_leg->trig_number = 3;
//        right_leg->trig_time = millis();
//        right_leg->Approve_trigger = false;
//        right_leg->trig3_counter += 1;
//        *(data_to_send_point) = right_leg->trig3_counter;
//      }
//      send_command_message('[', data_to_send_point, 1);
//      break;
//
//    case ']':// SS 8/6/2020
//      if (Trigger_left) {
//        left_leg->trig_number = 4;
//        left_leg->trig_time = millis();
//        left_leg->Approve_trigger = false;
//        left_leg->trig4_counter += 1;
//        *(data_to_send_point) = left_leg->trig4_counter;
//      } else {
//        right_leg->trig_number = 4;
//        right_leg->trig_time = millis();
//        right_leg->Approve_trigger = false;
//        right_leg->trig4_counter += 1;
//        *(data_to_send_point) = right_leg->trig4_counter;
//      }
//      send_command_message(']', data_to_send_point, 1);
//      break;
//
//    case 'r':// SS 8/6/2020
//      Trigger_left = false;
//      right_leg->Approve_trigger = false;
//      break;
//
//    case 's':// SS 8/6/2020
//      Trigger_left = true;
//      left_leg->Approve_trigger = false;
//      break;



    case 'Q':
      *(data_to_send_point) = left_leg->fsr_percent_thresh_Toe;
      *(data_to_send_point + 1) = right_leg->fsr_percent_thresh_Toe;
      send_command_message('Q', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      break;


    // TN 6/13/19
    case 'R':                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->fsr_percent_thresh_Toe, holdOnPoint, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&right_leg->fsr_percent_thresh_Toe, holdOnPoint + 8, 8);
      left_leg->p_steps->fsr_percent_thresh_Toe = left_leg->fsr_percent_thresh_Toe;
      right_leg->p_steps->fsr_percent_thresh_Toe = right_leg->fsr_percent_thresh_Toe;
      break;

    case 'T':
      left_leg->p_steps->count_plant = 0;
      left_leg->p_steps->n_steps = 0;
      left_leg->p_steps->flag_start_plant = false;
      left_leg->p_steps->flag_take_average = false;
      left_leg->p_steps->flag_N3_adjustment_time = false;
      left_leg->p_steps->flag_take_baseline = false;
      left_leg->p_steps->torque_adj = false;
      left_leg->N3 = N3;
      break;

    case 't':
      right_leg->p_steps->count_plant = 0;
      right_leg->p_steps->n_steps = 0;
      right_leg->p_steps->flag_start_plant = false;
      right_leg->p_steps->flag_take_average = false;
      right_leg->p_steps->flag_N3_adjustment_time = false;
      right_leg->p_steps->flag_take_baseline = false;
      right_leg->p_steps->torque_adj = false;
      right_leg->N3 = N3;
      break;

    case 'I':
      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint;
      left_leg->p_steps->torque_adj = false;
      break;

    case 'i':

      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint;
      right_leg->p_steps->torque_adj = false;
      break;

    case 'W':                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&app, &holdon, 8);
      left_leg->zero = zero + app;
      break;

    case 'X':                                         //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&app, &holdon, 8);
      right_leg->zero = zero + app;
      break;

    // TN 6/13/19
    case '{': // Receive Left Gain from GUI                                          //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->Prop_Gain,  holdOnPoint, 8);
      memcpy(&right_leg->Prop_Gain,  holdOnPoint + 8, 8);
      break;
    // TN 6/13/19
    case '}': // Send Left Gain to GUI
      *(data_to_send_point) = left_leg->Prop_Gain;
      *(data_to_send_point + 1) = right_leg->Prop_Gain;
      send_command_message('}', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case '+':

//      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
//      FLAG_ONE_TOE_SENSOR = false;
//      FLAG_BALANCE = true;
//      Old_Control_Mode = Control_Mode;
//      Control_Mode = 2;
//      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
//      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
//      FLAG_BALANCE = true;
      break;

    case '=':
//      FLAG_ONE_TOE_SENSOR = OLD_FLAG_ONE_TOE_SENSOR;
//      FLAG_ONE_TOE_SENSOR = true;
//      FLAG_BALANCE = false;
//      Control_Mode = Old_Control_Mode;
//      right_leg->p_steps->torque_adj = false;
//      left_leg->p_steps->torque_adj = false;
//      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint;
//      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint;
//      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
//      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
//      FLAG_BALANCE = false;
      break;


    case '.':
//      flag_auto_KF = 1;
//      left_leg->KF = 1;
//      right_leg->KF = 1;
//      left_leg->ERR = 0;
//      right_leg->ERR = 0;
      break;

    case ';':
//      flag_auto_KF = 0;
//      left_leg->KF = 1;
//      right_leg->KF = 1;
//      left_leg->ERR = 0;
//      right_leg->ERR = 0;
      break;

    case '#':
      Control_Mode = 3;
      break;


    case '^':
//      Control_Mode = Old_Control_Mode;
//      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR; //GO 4/23/19
//      FLAG_ONE_TOE_SENSOR = true; // TN 7/5/19
//      right_leg->p_steps->torque_adj = false;
//      left_leg->p_steps->torque_adj = false;
//      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint;
//      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint;
//      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
//      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
//      flag_id = false; // TN 05/13/19
//      flag_pivot = false; // TN 05/13/19
//      flag_resist = false; // GO 6/20/2020
//      Flag_Prop_Ctrl = false; // TN 04/29/19
      break;

    case 'B':
      if (FLAG_BIOFEEDBACK == true) {
        *(data_to_send_point) = right_leg->Heel_Strike_baseline;
        send_command_message('B', data_to_send_point, 1);

      } else if (FLAG_BALANCE == true) {

        *(data_to_send_point) = left_leg->FSR_Toe_Steady_Balance_Baseline * left_leg->Steady_multiplier;
        *(data_to_send_point + 1) = left_leg->FSR_Heel_Steady_Balance_Baseline * left_leg->Steady_multiplier;
        *(data_to_send_point + 2) = right_leg->FSR_Toe_Steady_Balance_Baseline * right_leg->Steady_multiplier;
        *(data_to_send_point + 3) = right_leg->FSR_Heel_Steady_Balance_Baseline * right_leg->Steady_multiplier;

        *(data_to_send_point + 4) = left_leg->FSR_Toe_Balance_Baseline * left_leg->Dynamic_multiplier;
        *(data_to_send_point + 5) = left_leg->FSR_Heel_Balance_Baseline * left_leg->Dynamic_multiplier;
        *(data_to_send_point + 6) = right_leg->FSR_Toe_Balance_Baseline * right_leg->Dynamic_multiplier;
        *(data_to_send_point + 7) = right_leg->FSR_Heel_Balance_Baseline * right_leg->Dynamic_multiplier;

        send_command_message('B', data_to_send_point, 8);

      }
      else {
        left_leg->baseline_value = left_leg->p_steps->plant_peak_mean;
        right_leg->baseline_value = right_leg->p_steps->plant_peak_mean;
        *(data_to_send_point) = left_leg->p_steps->plant_peak_mean;
        *(data_to_send_point + 1) = right_leg->p_steps->plant_peak_mean;
        send_command_message('B', data_to_send_point, 2);
      }


      break;

    case 'b':
      left_leg->FSR_baseline_FLAG = 1;
      right_leg->FSR_baseline_FLAG = 1;
      left_leg->p_steps->count_plant_base = 0;
      right_leg->p_steps->count_plant_base = 0;
      right_leg->p_steps->flag_start_plant = false;
      left_leg->p_steps->flag_start_plant = false;
      break;

    case '&':
      FLAG_BALANCE_BASELINE = 1;

      startTime = millis();
      Control_Mode = Old_Control_Mode;// you cannot calibrate if your doing something
      left_leg->FSR_Toe_Balance_Baseline = 0;
      right_leg->FSR_Toe_Balance_Baseline = 0;
      left_leg->FSR_Heel_Balance_Baseline = 0;
      right_leg->FSR_Heel_Balance_Baseline = 0;
      count_balance = 0;
      break;

    case 'J':
      FLAG_STEADY_BALANCE_BASELINE = 1;

      startTime = millis();
      Control_Mode = Old_Control_Mode;// you cannot calibrate if your doing something
      left_leg->FSR_Toe_Steady_Balance_Baseline = 0;
      right_leg->FSR_Toe_Steady_Balance_Baseline = 0;
      left_leg->FSR_Heel_Steady_Balance_Baseline = 0;
      right_leg->FSR_Heel_Steady_Balance_Baseline = 0;
      count_steady_baseline = 0;
      break;


    case '|':
      FLAG_AUTO_RECONNECT_BT = true;
      break;

    case '@':
      FLAG_AUTO_RECONNECT_BT = false;
      break;


    case 'V':
      *(data_to_send_point) = left_leg->Steady_multiplier;
      send_command_message('V', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'v':                                          //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->Steady_multiplier, &holdon, 8);
      memcpy(&right_leg->Steady_multiplier, &holdon, 8);
      break;

    case 'A':
      *(data_to_send_point) = left_leg->Dynamic_multiplier;
      send_command_message('A', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'a':                                          //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->Dynamic_multiplier, &holdon, 8);
      memcpy(&right_leg->Dynamic_multiplier, &holdon, 8);
      break;


    case '/':
      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      FLAG_ONE_TOE_SENSOR = false;
      FLAG_BIOFEEDBACK = true;
      right_leg->BIO_BASELINE_FLAG = false;
      break;


    case 'y':
      FLAG_ONE_TOE_SENSOR = OLD_FLAG_ONE_TOE_SENSOR;
      FLAG_BIOFEEDBACK = false;
      if (left_leg->state == 2) left_leg->state = 1;
      if (right_leg->state == 2) right_leg->state = 1;
      break;


    case 'u':                                        //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->BioFeedback_desired, &holdon, 8);
      right_leg->BioFeedback_desired = left_leg->BioFeedback_desired;
      break;

    case '*':                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&treadmill_speed, &holdon, 8); //YF
      break;

    // Optimization ------------------------------------------------


    case '$':
//      if (Flag_HLO) {
//        memcpy(&left_leg->Setpoint_Ankle_Opt, holdOnPoint, 8); //Ankle torque setpoint for bang-bang
//        memcpy(&left_leg->T_Opt_p, holdOnPoint + 8, 8);       //Ankle torque rise time percentage for bang-bang
//        right_leg->Setpoint_Ankle_Opt = -left_leg->Setpoint_Ankle_Opt;
//        right_leg->T_Opt_p = left_leg->T_Opt_p;
//        Serial.println(left_leg->FLAG_UPDATE_VALUES);
//        left_leg->FLAG_UPDATE_VALUES = true;
//        right_leg->FLAG_UPDATE_VALUES = true;
//        Serial.println(left_leg->FLAG_UPDATE_VALUES);
//        Serial.print("Received these values from HLO : ");
//        Serial.print(left_leg->Setpoint_Ankle_Opt);
//        Serial.print(" , ");
//        Serial.println(left_leg->T_Opt_p);
//
//        left_leg->activate_in_3_steps = 1;
//        left_leg->num_3_steps = 0;
//        left_leg->first_step = 1;
//        left_leg->start_step = 0;
//
//        right_leg->activate_in_3_steps = 1;
//        right_leg->num_3_steps = 0;
//        right_leg->first_step = 1;
//        right_leg->start_step = 0;
//      }
    break;  

    case '"':
//      if (Flag_HLO) {
//        left_leg->Previous_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
//        right_leg->Previous_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
//        left_leg->Previous_Dorsi_Setpoint_Ankle = left_leg->Dorsi_Setpoint_Ankle;
//        right_leg->Previous_Dorsi_Setpoint_Ankle = right_leg->Dorsi_Setpoint_Ankle;
//        memcpy(&left_leg->Setpoint_Ankle, holdOnPoint, 8); //HLO proportional control setpoint
//        right_leg->Setpoint_Ankle = -left_leg->Setpoint_Ankle;
//        left_leg->activate_in_3_steps = 1;
//        left_leg->num_3_steps = 0;
//        left_leg->first_step = 1;
//        left_leg->start_step = 0;
//        right_leg->activate_in_3_steps = 1;
//        right_leg->num_3_steps = 0;
//        right_leg->first_step = 1;
//        right_leg->start_step = 0;
//        left_leg->FSR_baseline_FLAG = 1;                      //Retake the baseline every new setpoint
//        right_leg->FSR_baseline_FLAG = 1;
//      }
       break;



    // ------------------------------------------------------------


    case ':':
      left_leg->BioFeedback_Baseline_flag = false;
      right_leg->BioFeedback_Baseline_flag = false;
      left_leg->BIO_BASELINE_FLAG = true;
      right_leg->BIO_BASELINE_FLAG = true;
      left_leg->Heel_Strike = 0;
      right_leg->Heel_Strike = 0;
      left_leg->Heel_Strike_Count = 0;
      right_leg->Heel_Strike_Count = 0;
      left_leg->score = 0;
      right_leg->score = 0;
      break;

    case 'U':
      data_to_send_point[0] = (double) VERSION;
      data_to_send_point[1] = (double) BOARD_VERSION;
      send_command_message('U', data_to_send_point, 2);
      break;

    case 'z':
      flag_motor_error_check = !flag_motor_error_check;
      data_to_send_point[0] = flag_motor_error_check;
      send_command_message('z', data_to_send_point, 1);
      break;

    case 'g':                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->p_steps->plant_peak_mean_temp, holdOnPoint, 8);              // send an old value of plant_peak_mean to teensy  // TN 04-26-2019
      //delay(10);
      //receiveVals(8);
      memcpy(&right_leg->p_steps->plant_peak_mean_temp, holdOnPoint + 8, 8);//added          // send an old value of plant_peak_mean to teensy  // TN 04-26-2019
      //delay(10);
      //receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      //      memcpy(&left_leg->Curr_Combined, holdOnPoint + 16, 8);
      //      //      delay(10);
      //      //      receiveVals(8);
      //      memcpy(&right_leg->Curr_Combined, holdOnPoint + 24, 8);//added
      //      //      delay(10);
      //      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      //      memcpy(&left_leg->fsr_Combined_peak_ref, holdOnPoint + 32, 8);
      //      //      delay(10);
      //      //      receiveVals(8);
      //      memcpy(&right_leg->fsr_Combined_peak_ref, holdOnPoint + 40, 8);//added
      //      //      delay(10);
      //      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      //      memcpy(&left_leg->fsr_Toe_peak_ref, holdOnPoint + 48, 8);
      //      //      delay(10);
      //      //      receiveVals(8);
      //      memcpy(&right_leg->fsr_Toe_peak_ref, holdOnPoint + 56, 8);//added
      //      //      delay(10);
      //      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      //      memcpy(&left_leg->fsr_Heel_peak_ref, holdOnPoint + 64, 8);
      //      //      delay(10);
      //      //      receiveVals(8);
      //      memcpy(&right_leg->fsr_Heel_peak_ref, holdOnPoint + 72, 8);//added
      //      delay(10);
      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->torque_calibration_value, holdOnPoint + 16, 8);
      //      delay(10);
      //      receiveVals(8);
      memcpy(&right_leg->torque_calibration_value, holdOnPoint + 24, 8);//added


      //      Serial.println("Old left_leg->p_steps->plant_peak_mean_temp");
      //      Serial.println(left_leg->p_steps->plant_peak_mean_temp);
      //      Serial.println("Old right_leg->p_steps->plant_peak_mean_temp");
      //      Serial.println(right_leg->p_steps->plant_peak_mean_temp);
      //      Serial.println(left_leg->Curr_Combined);
      //      Serial.println(right_leg->Curr_Combined);
      //      Serial.println(left_leg->fsr_Combined_peak_ref);
      //      Serial.println(right_leg->fsr_Combined_peak_ref);
      //      Serial.println(left_leg->fsr_Toe_peak_ref);
      //      Serial.println(right_leg->fsr_Toe_peak_ref);
      //      Serial.println(left_leg->fsr_Heel_peak_ref);
      //      Serial.println(right_leg->fsr_Heel_peak_ref);
      //      Serial.println(left_leg->torque_calibration_value);
      //      Serial.println(right_leg->torque_calibration_value);


      break;

    case 'j':   // TN

//      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
//      FLAG_ONE_TOE_SENSOR = true;
//      Old_Control_Mode = Control_Mode;
//      Control_Mode = 5;  // Averaged torque profiles


      break;

    case 'o':   // TN 6/7/19

//      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
//      FLAG_ONE_TOE_SENSOR = true;
//      Old_Control_Mode = Control_Mode;
//      Control_Mode = 100;
//
//      right_leg->p_steps->torque_adj = false;
//      left_leg->p_steps->torque_adj = false;
//      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint;
//      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint;
//      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
//      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;

      break;
  }
  cmd_from_Gui = 0;
}
