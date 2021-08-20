/* Defines class to determine ambulation state.
   Two states*/

#ifndef IMU_H
#define IMU_H

/* 
 * 
 */

#include <Arduino_LSM9DS1.h>
#include "ema_filter.h"
#include "motor_utils.h"

//Typedef for holding callback functions
typedef void (*cb_t)();
//State Management
enum States {Standing, Walking};

class Ambulation_SM {
  public:
  States last_state = Standing;
  void init() {
    if (!IMU.begin()) {
       while (1);
    }
  }

  inline void tick(unsigned int steps) {
    float x, y, z;
    x = 2;
    if (IMU.readAcceleration(x, y, z)) {
      y = -y;
      correct_sagittal_angle(y, z);
      compute_resultant(x, y, z);
      //estimate_walking_speed(z);
      track_steps(steps);
      update_threshold();
      check_state();
    }
  }

  inline void attach_fe_cb(cb_t cb) {
    fe_cb = cb;
    if ((fe_cb != NULL) && (re_cb != NULL)) cbs_set = true;
  }
  inline void attach_re_cb(cb_t cb) {
    re_cb = cb;
    if ((fe_cb != NULL) && (re_cb != NULL)) cbs_set = true;
  }
 
  private:
  //Const(s)
  const float thrsh_offset_k = 0.01f;
  const unsigned long int reset_duration_k = 1500;  //millis
  const float walking_bias_k = 0.1f;

  //Resultant acceleration
  float resultant = 0;

  //Step tracking
  bool bias_tracking = false;
  unsigned int last_steps = 0;

  //filtered angle
  float filtered_angle = 0;

  //Estimated speed [m/s]
  float est_speed = 0;
  float filtered_speed = 0;

  //Dynamic Threshold
  float threshold = 0.0f;
  float avg_res = 0;
  float alpha_for_filter = 0.8;
  float alpha_for_avg = 0.2;
  float st_dev = 0;
   
  //Callbacks/cb management
  /* Falling edge is defined as walking to standing */
  cb_t fe_cb = NULL;
  cb_t re_cb = NULL;
  bool cbs_set = false;

  //Timer Value
  unsigned long int last_reset;
  unsigned long int last_step_reset;
 
  //Private Functions
  inline void compute_resultant(float x, float y, float z) {
    resultant = ema_with_context(resultant, (sqrt(z*z + y*y + x*x)), alpha_for_filter);
  }

  inline void estimate_walking_speed(float coronal_acc) {
    coronal_acc *= 9.81*3.6;
    est_speed += coronal_acc * CONTROL_TIME_STEP;
    filtered_speed = ema_with_context(filtered_speed, est_speed, 0.9);
    left_torque = filtered_speed;
  }

  inline void track_steps(unsigned int steps) {
    unsigned long int start = millis();
    if (steps > last_steps) {
      last_step_reset = start;
      bias_tracking = true;
      left_torque = 1;
    }
    unsigned long int delta = start - last_step_reset;
    if (delta > reset_duration_k) {
      bias_tracking = false;
      left_torque = 0;
    }
    last_steps = steps;
  }

  inline void update_threshold() {
    avg_res = ema_with_context(avg_res, resultant, alpha_for_avg);
    if (bias_tracking) {
      threshold = (thrsh_offset_k * walking_bias_k) + avg_res;
    } else {
      threshold = (thrsh_offset_k / walking_bias_k) + avg_res;
    }
  }

  inline void check_state() {
    //Serial.print(resultant);
    //Serial.print('\t');
    //Serial.println(threshold);
    right_torque = resultant;
    right_setpoint = threshold;
    /* If there is a large acceleration, the user is walking */
    uint32_t start = millis();
    if (resultant > threshold) {
      last_reset = start;
      if (last_state == Standing) {
        last_state = Walking;
        re_cb();
      }
    }
    /* If the Timer is not Reset, user is standing */
    uint32_t delta = start - last_reset;
    if (delta > reset_duration_k) {
      if (last_state == Walking) {
        last_state = Standing;
        fe_cb();
      }
    }
  }

  void correct_sagittal_angle(float& y, float& z) {
    /* Dont divide by zero */
    float acc_angle;
    if (abs(y) < 0.00001) {
      acc_angle = float(HALF_PI);
    } else {
      acc_angle = float(atan2(z, y));
    }
    //filtered_angle = ema_with_context(filtered_angle, acc_angle, alpha_for_filter);
    y -= cos(acc_angle);
    z -= sin(acc_angle);
  }
}; //End Class

namespace Amb_SM_cbs {
  void upon_walking() {
    change_motor_stateless(true);
  }

  void upon_standing() {
    change_motor_stateless(false);
  }
}

#endif
