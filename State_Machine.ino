#include "State_Machine.h"

#include "Leg.h"
#include "Reference_ADJ.h"

//--------------------NEW FUNCTIONS-----------------
// Before state 3 in case of balance control was activated just in case of not negligible value 
// of the force on the toe i.e. if you just walk on the heel you never feel the balance.
// Right now the state 3 is activate also in case of heel contact and the state 1 is activated
// in case of no contact of both the sensors

void state_machine(Leg* leg)
{
  boolean foot_on_fsr;
  if (FLAG_TWO_TOE_SENSORS) {
    foot_on_fsr = leg->p_steps->curr_voltage > leg->fsr_percent_thresh_Toe * leg->fsr_Combined_peak_ref;
  } else if (FLAG_BALANCE) {
    foot_on_fsr = (leg->FSR_Toe_Average > leg->fsr_percent_thresh_Toe * leg->FSR_Toe_Balance_Baseline) ||
      (leg->FSR_Heel_Average > leg->fsr_percent_thresh_Toe * leg->FSR_Heel_Balance_Baseline);
  } else {
    foot_on_fsr =leg->p_steps->curr_voltage > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref;
  }
  state_machine_generic(leg, foot_on_fsr);

}

void state_machine_generic(Leg* leg, boolean foot_on_fsr){
  switch (leg->state)
  {
  case SWING: //Swing
    state_machine_swing(leg, foot_on_fsr);
    break;
  case LATE_STANCE: //Late Stance
    state_machine_late_stance(leg, foot_on_fsr);
    break;
  }//end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Trq_time_volt == 2 || Trq_time_volt == 3) && leg->state == LATE_STANCE) {
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }


}

void state_machine_swing(Leg* leg, boolean foot_on_fsr){
  // This flag enables the "set to zero" procedure for the left ankle.
  // When you're for at least 3 seconds in the same state, the torque reference is set to zero

  if (leg->set_2_zero == 1) {
    leg->set_2_zero = 0;
    leg->One_time_set_2_zero = 1;
  }
  leg->determineState(foot_on_fsr);
}

void state_machine_late_stance(Leg* leg, boolean foot_on_fsr){
  if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
    leg->sigm_done = true;
    leg->Old_PID_Setpoint = leg->PID_Setpoint;
    leg->state_old = leg->state;
    leg->New_PID_Setpoint = 0;
    leg->One_time_set_2_zero = 0;
    leg->Previous_Setpoint_Ankle = 0;
    leg->PID_Setpoint = 0;
    leg->Setpoint_Ankle_Pctrl = 0;
  }
  leg->determineState(foot_on_fsr);

}
