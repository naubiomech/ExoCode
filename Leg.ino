#include "Leg.h"
#include "Pins.h"
#include "Shaping_Functions.h"

Leg::Leg(LegPins* legPins){
  this->foot_fsrs = new FSRGroup(legPins->fsr_pins, legPins->fsr_count);
  this->motor_count = legPins->motor_count;
  this->motors = new Motor*[motor_count];
  for (int i = 0; i < legPins->motor_count; i++){
    this->motors[i] = new Motor(legPins->motor_pins[i]);
  }
}

double Leg::getBalanceReference(){
  return foot_fsrs->getBalanceReference * Prop_Gain;
}

void Leg::calibrateFSRs(){
  foot_fsrs->calibrate();
}

void Leg::startTorqueCalibration(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->startTorqueCalibration();
  }
}

void Leg::updateTorqueCalibration(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->updateTorqueCalibration();
  }
}

void Leg::endTorqueCalibration(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->endTorqueCalibration();
  }
}

bool Leg::checkMotorErrors(){
  for(int i = 0; i < motor_count;i++){
    if(motors[i]->hasErrored()){
      return true;
    }
  }
  return false;
}

void Leg::autoKF(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->autoKF(state);
  }
}

void Leg::adjustControl(){
  this->N3 = Ctrl_ADJ(this->state, this->state_old, this->p_steps,
                      this->N3, this->New_PID_Setpoint, &this->Setpoint_Ankle,
                      &this->Setpoint_Ankle_Pctrl, Trq_time_volt, this->Prop_Gain,
                      this->FSR_baseline_FLAG, &this->FSR_Ratio, &this->Max_FSR_Ratio);
}

void Leg::resetStartingParameters(){
  this->p_steps->count_plant = 0;
  this->p_steps->n_steps = 0;
  this->p_steps->flag_start_plant = false;
  this->p_steps->flag_take_average = false;
  this->p_steps->flag_N3_adjustment_time = false;
  this->p_steps->flag_take_baseline = false;
  this->p_steps->torque_adj = false;

  this->N3 = N3;
  this->N2 = N2;
  this->N1 = N1;

  this->p_steps->perc_l = 0.5;
  this->activate_in_3_steps = 1;
  this->Previous_Setpoint_Ankle = 0;

  this->coef_in_3_steps = 0;
  this->num_3_steps = 0;

  this->first_step = 1;
}


void Leg::setZeroIfSteadyState(){
  if (this->flag_1 == 0) {
    this->flag_1 = 1;
    this->time_old_state = this->state;
  }
  if (this->state != this->time_old_state) {
    this->flag_1 = 0;
    this->stateTimerCount = 0;
  } else {
    if (this->stateTimerCount >= 5 / 0.002) {
      if (this->store_N1 == 0) {
        Serial.println("Steady state, setting to 0Nm , Change N1");
        this->set_2_zero = 1;
        this->store_N1 = 1;
        this->activate_in_3_steps = 1;
        this->num_3_steps = 0;
        this->first_step = 1;
        this->start_step = 0;
      }
    } else {
      this->stateTimerCount++;
      if (this->store_N1) {
        this->set_2_zero = 0;
        this->store_N1 = 0;
      }
    }
  }
}

void Leg::determineState(boolean foot_on_fsr){
  int new_state;
  if (swing_state_threshold->getState((double) foot_on_fsr)){
    new_state = SWING;
  } else {
    new_state = LATE_STANCE;
  }

  if (new_state != this->state){
    this->sigm_done = true;
    this->Old_PID_Setpoint = this->PID_Setpoint;
    this->New_PID_Setpoint = this->Previous_Setpoint_Ankle +
      (this->Setpoint_Ankle - this->Previous_Setpoint_Ankle) * this->coef_in_3_steps;

    this->state_old = this->state;
    this->state = state;
    this->state_count_13 = 0;
    this->state_count_31 = 0;
    if (state == LATE_STANCE){
      if (abs(this->Dorsi_Setpoint_Ankle) > 0) {
        this->Old_PID_Setpoint = 0;
      } else {
        this->Previous_Dorsi_Setpoint_Ankle = 0;
      }
    }
  }
}

bool Leg::determine_foot_on_ground(){
  boolean foot_on_fsr;
  if (FLAG_TWO_TOE_SENSORS) {
    foot_on_fsr = this->p_steps->curr_voltage > this->fsr_percent_thresh_Toe * this->fsr_Combined_peak_ref;
  } else if (FLAG_BALANCE) {
    foot_on_fsr = (this->FSR_Toe_Average > this->fsr_percent_thresh_Toe * this->FSR_Toe_Balance_Baseline) ||
      (this->FSR_Heel_Average > this->fsr_percent_thresh_Toe * this->FSR_Heel_Balance_Baseline);
  } else {
    foot_on_fsr =this->p_steps->curr_voltage > this->fsr_percent_thresh_Toe * this->fsr_Toe_peak_ref;
  }
  return foot_on_fsr;
}

void Leg::applyStateMachine(){

  if (this->set_2_zero){
    switch (this->state) {
    case SWING:
      this->set_2_zero = 0;
      this->One_time_set_2_zero = 1;
      break;
    case LATE_STANCE:
      if (this->One_time_set_2_zero) {
        this->sigm_done = true;
        this->Old_PID_Setpoint = this->PID_Setpoint;
        this->state_old = this->state;
        this->New_PID_Setpoint = 0;
        this->One_time_set_2_zero = 0;
        this->Previous_Setpoint_Ankle = 0;
        this->PID_Setpoint = 0;
        this->Setpoint_Ankle_Pctrl = 0;
      }
      break;
    }
  }

  bool foot_on_ground = determine_foot_on_ground();
  this->determineState(foot_on_fsr);
  ref_step_adj(this);

  if ((Trq_time_volt == 2 || Trq_time_volt == 3) && this->state == LATE_STANCE) {
    this->PID_Setpoint = this->Setpoint_Ankle_Pctrl;
  } else if (N1 < 1 || N2 < 1 || N3 < 1) {
    this->PID_Setpoint = this->New_PID_Setpoint;
  } else {
    // Create the smoothed reference and call the PID
    sigmoidCurveSetpoint();
  }
}

void Leg::sigmoidCurveSetpoint(){
  long sig_time = millis();
  if ((sig_time - this->sig_time_old) > 1) {

    this->sig_time_old = sig_time;

    if ((abs(this->New_PID_Setpoint - this->PID_Setpoint) > 0.1) && (this->sigm_done)) {

      this->sigm_done = false;
      this->n_iter = 0;
      int N_step;

      if (this->state == LATE_STANCE) {
        N_step = N3;
      } else if (this->state == SWING) {
        N_step = N1;
      }
      double exp_mult = round((10 / Ts) / (N_step - 1));
    }

    if (this->n_iter < N_step) {
      this->PID_Setpoint = calculatePIDSetpointSigm(New_PID_Setpoint, this->Old_PID_Setpoint,
                                                    Ts, exp_mult, this->n_iter, N_step);
      this->n_iter++;
    } else {
      this->sigm_done = true;
    }
  }
}

void Leg::measureSensors(){
  this->ankle_motor->measureTorque();
  this->ankle_motor->measureError();
  this->fsrs->measureForce();

  this->p_steps->torque_average = this->ankle_motor->getTorque();
}

void Leg::takeFSRBaseline(){
  if (FSR_baseline_FLAG){
    take_baseline(state, state_old, p_steps, &FSR_baseline_FLAG);
  }
}

bool Leg::applyTorque(){
  if (!ankle_motor->applyTorque(state)){
    state = 9;
    return false;
  }
  return true;
}
