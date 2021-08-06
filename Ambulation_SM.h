/* Defines class to determine ambulation state.
   Two states*/

#ifndef IMU_H
#define IMU_H

/*
 * 
 */

#include <Arduino_LSM9DS1.h>
#include "ema_filter.h"
//Typedef for holding callback functions
typedef void (*cb_t)();

class Ambulation_SM {
 
  public:
  void init() {
    if (!IMU.begin()) {
       while (1);
    }
  }

  inline void tick() {
    float x, y, z;
    if (IMU.accelerationAvailable()) {
      if (IMU.readAcceleration(x, y, z)) {
        y = y * -1;
        correct_sagittal_angle(y, z);
        compute_resultant(x, y, z);
        update_threshold();
        check_state();
      }
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
  const float thrsh_offset_k = 0.075f;
  const unsigned long int reset_duration_k = CONTROL_LOOP_HZ * 3;

  //Resultant acceleration
  float resultant = 0;

  //Estimated speed [m/s]
  float est_speed = 0;

  //Dynamic Threshold
  float threshold = 0.0f;
  float avg_res = 0;
  float alpha_for_filter = 0.2;
  float alpha_for_avg = 0.05;
  float st_dev = 0;

  //State Management
  enum States {Standing, Walking};
  States last_state = Standing;
   
  //Callbacks/cb management
  /* Falling edge is defined as walking to standing */
  cb_t fe_cb = NULL;
  cb_t re_cb = NULL;
  bool cbs_set = false;

  //Timer Value
  unsigned long int last_reset;
 
  //Private Functions
  inline void compute_resultant(float x, float y, float z) {
    /* Must subtract by abs(x) because the toy model is two dimensional */
    resultant = ema_with_context(resultant, (sqrt(y*y + z*z) - abs(x)), alpha_for_filter);
  }

  inline void update_threshold() {
    avg_res = ema_with_context(avg_res, resultant, alpha_for_avg);
    threshold = thrsh_offset_k + abs(avg_res);
  }

  inline void check_state() {
    /* If there is a large acceleration, the user is walking */
    if (abs(resultant) > threshold) {
      last_reset = millis();
      if (last_state == Standing) {
        last_state = Walking;
        re_cb();
      }
    }
    /* If the Timer is not Reset, user is standing */
    unsigned long int delta = millis() - last_reset;
    if (delta > reset_duration_k) {
      if (last_state == Walking) {
        last_state = Standing;
        fe_cb();
      }
    }
  }

  void correct_sagittal_angle(float& y, float& z) {
    /* Dont devide by zero */
    float theta;
    if (abs(y) < 0.01) {
      theta = float(HALF_PI);
    } else {
      theta = float(atan2(z, y));
    }
    y -= cos(theta);
    z -= sin(theta);
  }
}; //End Class


#endif
