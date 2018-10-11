#include "Leg.h"

void Leg::initalize(){

  pinMode(this.pin_err, INPUT);
  pinMode(this.torque_sensor_ankle_pin, INPUT);

  analogWrite(this.motor_ankle_pin, zero);
  this.balance_pid.SetMode(AUTOMATIC);
  this.balance_pid.SetTunings(this.kp_balance, this.ki_balance, this.kd_balance);
  this.balance_pid.SetOutputLimits(-1500, 1500);
  this.balance_pid.SetSampleTime(PID_sample_time);

  this.ankle_pid.SetMode(AUTOMATIC);
  this.ankle_pid.SetTunings(this.kp_ankle, this.ki_ankle, this.kd_ankle);
  this.ankle_pid.SetOutputLimits(-1500, 1500);
  this.ankle_pid.SetSampleTime(PID_sample_time);

  this.p_steps->fsr_Toe = this.fsr_sense_Toe;
  this.zero = zero;
}

double Leg::getBalanceReference(){
  return foot_fsrs->getBalanceReference * Prop_Gain;
}

void Leg::calibrateFSRs(){
  foot_fsrs->calibrate();
}
void Leg::startTorqueCalibration(){
  ankle_motor->startTorqueCalibration;
}

void Leg::updateTorqueCalibration(){
  ankle_motor->updateTorqueCalibration;
}

void Leg::endTorqueCalibration(){
  ankle_motor->endTorqueCalibration;
}

bool Leg::checkMotorErrors(){
  return this.ankle_motor->hasErrored();
}

void Leg::autoKF(){
  ankle_motor->autoKF(state);
}

void Leg::adjustControl(){
  leg->N3 = Ctrl_ADJ(leg->state, leg->state_old, leg->p_steps,
                     leg->N3, leg->New_PID_Setpoint, &leg->Setpoint_Ankle,
                     &leg->Setpoint_Ankle_Pctrl, Trq_time_volt, leg->Prop_Gain,
                     leg->FSR_baseline_FLAG, &leg->FSR_Ratio, &leg->Max_FSR_Ratio);
}

void Leg::resetStartingParameters(){
  leg->p_steps->count_plant = 0;
  leg->p_steps->n_steps = 0;
  leg->p_steps->flag_start_plant = false;
  leg->p_steps->flag_take_average = false;
  leg->p_steps->flag_N3_adjustment_time = false;
  leg->p_steps->flag_take_baseline = false;
  leg->p_steps->torque_adj = false;

  leg->N3 = N3;
  leg->N2 = N2;
  leg->N1 = N1;

  leg->p_steps->perc_l = 0.5;
  leg->activate_in_3_steps = 1;
  leg->Previous_Setpoint_Ankle = 0;

  leg->coef_in_3_steps = 0;
  leg->num_3_steps = 0;

  leg->first_step = 1;
}


void Leg::setZeroIfSteadyState(){
  if (leg->flag_1 == 0) {
    leg->flag_1 = 1;
    leg->time_old_state = leg->state;
  }
  if (leg->state != leg->time_old_state) {
    leg->flag_1 = 0;
    leg->stateTimerCount = 0;
  } else {
    if (leg->stateTimerCount >= 5 / 0.002) {
      if (leg->store_N1 == 0) {
        Serial.println("Steady state, setting to 0Nm , Change N1");
        leg->set_2_zero = 1;
        leg->store_N1 = 1;
        leg->activate_in_3_steps = 1;
        leg->num_3_steps = 0;
        leg->first_step = 1;
        leg->start_step = 0;
      }
    } else {
      leg->stateTimerCount++;
      if (leg->store_N1) {
        leg->set_2_zero = 0;
        leg->store_N1 = 0;
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

  if (new_state != leg->state){
    leg->sigm_done = true;
    leg->Old_PID_Setpoint = leg->PID_Setpoint;
    leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle +
      (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;

    leg->state_old = leg->state;
    leg->state = state;
    leg->state_count_13 = 0;
    leg->state_count_31 = 0;
    if (state == LATE_STANCE){
      if (abs(leg->Dorsi_Setpoint_Ankle) > 0) {
        leg->Old_PID_Setpoint = 0;
      } else {
        leg->Previous_Dorsi_Setpoint_Ankle = 0;
      }
    }
  }
}

void Leg::applyStateMachine(){
  state_machine(this);
}

void Leg::measureSensors(){
  leg->ankle_motor->measureTorque();
  leg->ankle_motor->measureError();
  leg->fsrs->measureForce();

  leg->p_steps->torque_average = leg->ankle_motor->getTorque();
}

void Leg::takeFSRBaseline(){
  if (FSR_baseline_FLAG){
    take_baseline(state, state_old, p_steps, &FSR_baseline_FLAG);
  }
}

bool Leg::applyTorque(){
  return ankle_motor->applyTorque(state);
}
