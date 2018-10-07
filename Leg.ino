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
  set_to_zero_if_leg_in_steady_state(this);
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

void initialize_left_leg(Leg* left_leg) {
  left_leg->pin_err = MOTOR_ERROR_LEFT_ANKLE_PIN;
  left_leg->fsr_sense_Heel = FSR_SENSE_LEFT_HEEL_PIN;
  left_leg->fsr_sense_Toe = FSR_SENSE_LEFT_TOE_PIN;
  left_leg->torque_address = 0;
  left_leg->address_FSR = 18;
  left_leg->p_steps = &val_L;
  left_leg->torque_sensor_ankle_pin = TORQUE_SENSOR_LEFT_ANKLE_PIN;
  left_leg->motor_ankle_pin = MOTOR_LEFT_ANKLE_PIN;
  left_leg->baseline_address = address_params + 105 + 5;
  initialize_leg(left_leg);
}

void initialize_right_leg(Leg* right_leg) {
  right_leg->pin_err = MOTOR_ERROR_RIGHT_ANKLE_PIN;
  right_leg->fsr_sense_Heel = FSR_SENSE_RIGHT_HEEL_PIN;
    right_leg->fsr_sense_Toe = FSR_SENSE_RIGHT_TOE_PIN;
    right_leg->torque_address = 9;
    right_leg->address_FSR = 36;
    right_leg->p_steps = &val_R;
    right_leg->torque_sensor_ankle_pin = TORQUE_SENSOR_RIGHT_ANKLE_PIN;
    right_leg->motor_ankle_pin = MOTOR_RIGHT_ANKLE_PIN;
    right_leg->baseline_address = address_params + 105 + 5 + 9;
    initialize_leg(right_leg);
  }
