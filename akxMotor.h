#ifndef AKXM_H
#define AKXM_H

#include <mbed.h>
#include "mcp2515.h"
#include "board.h"


#define AK80

//Motor max/mins
#define P_MIN -12.5f
#define P_MAX 12.5f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f

#ifdef AK80
#define T_MIN -18.0f
#define T_MAX 18.0f
#define V_MIN -25.64f
#define V_MAX 25.64f
#endif

#ifdef AK60
#define T_MIN -9.0f
#define T_MAX 9.0f
#define V_MIN -41.87f
#define V_MAX 41.87f
#endif


//Weigth Defaults
#define KP_DEF 0.0f
#define KD_DEF 0.0f

constexpr canid_t L_ID = 1;
constexpr canid_t R_ID = 2;

typedef struct {
  canid_t id;
  float pos;
  float vel;
  float tor;
  float kp;
  float kd;
} motor_frame_t;


class akxMotor {
  public: 
    inline bool init() {
      pinMode(OE, OUTPUT);
      digitalWrite(OE, HIGH);
      delay(10);

      mcp2515.init();
      mcp2515.reset();
      mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
      mcp2515.setNormalMode();

      scaling_factor = 2*T_MAX / 4096;
    }

    /* Takes the voltage output of PID, maps it to a torque and sends it */
    inline void map_and_apply(canid_t id, float vol) {
      motor_frame_t out_frame;
      out_frame.id = id;
      out_frame.pos = 0;
      out_frame.vel = 0;
      out_frame.kp = 0;
      out_frame.kd = 0;

      float torque = (vol)*scaling_factor;
      out_frame.tor = torque;
      sendCAN(&out_frame);
    }

    /* This function will wait for timeout to execute, if timeout or error, return ID 0. Timout is in millis */
    inline bool updateFrame(motor_frame_t* out_frame, float timeout = DEF_TIMEOUT) {
      motor_frame_t temp_frame;
      temp_frame.id = 0;
      temp_frame.pos = 0;
      temp_frame.vel = 0;
      temp_frame.tor = 0;
      temp_frame.kp = 0;
      temp_frame.kd = 0;
      float start = millis();
      while(true) {
        /* Read */
        MCP2515::ERROR err = readCAN(&temp_frame);

        /* Check for errors and message */
        if(err == MCP2515::ERROR_OK && temp_frame.id == out_frame->id) {
          out_frame->pos = temp_frame.pos;
          out_frame->vel = temp_frame.vel;
          out_frame->tor = temp_frame.tor;
//          if(temp_frame.tor != T_MAX) {
//            out_frame->tor = temp_frame.tor;
//          }
          return true;
        }

        /* Check for timeout */
        if((millis() - start) >= timeout) {
          return false;
        } //End timeout check
      } //End While
    } //End function

    inline void setZero(canid_t id) {
      /* Sets the current zero of the motor position
         NEEDS TESTING */
      struct can_frame out_frame;
      out_frame.can_id = id;
      out_frame.can_dlc = 8; 
      out_frame.data[0] = 0xFF;
      out_frame.data[1] = 0xFF;
      out_frame.data[2] = 0xFF;
      out_frame.data[3] = 0xFF;
      out_frame.data[4] = 0xFF;
      out_frame.data[5] = 0xFF;
      out_frame.data[6] = 0xFF;
      out_frame.data[7] = 0xFE;
      
      if (mcp2515.sendMessage(&out_frame) != MCP2515::ERROR_OK) {
        Serial.println("Set Zero Error!");
      }
    }

    inline void setMotorState(canid_t id, bool enable) {
      /* Turns the motor with id on or off */
      struct can_frame out_frame;
      out_frame.can_id = id;
      out_frame.can_dlc = 8; 
      out_frame.data[0] = 0xFF;
      out_frame.data[1] = 0xFF;
      out_frame.data[2] = 0xFF;
      out_frame.data[3] = 0xFF;
      out_frame.data[4] = 0xFF;
      out_frame.data[5] = 0xFF;
      out_frame.data[6] = 0xFF;
      if (enable) {
        out_frame.data[7] = 0xFC;
      } else {
        out_frame.data[7] = 0xFD;
      }
      if (mcp2515.sendMessage(&out_frame) != MCP2515::ERROR_OK) {
        Serial.println("Set State Error!");
      }
    }
    
    inline void sendCAN(motor_frame_t *frame) {
      //Constrain inputs
      float p_sat = constrain(frame->pos, P_MIN, P_MAX);
      float v_sat = constrain(frame->vel, V_MIN, V_MAX);
      float kp_sat = constrain(frame->kp, KP_MIN, KP_MAX);
      float kd_sat = constrain(frame->kd, KD_MIN, KD_MAX);
      float t_sat = constrain(frame->tor, T_MIN, T_MAX);
    
      //Convert floats to unsigned ints
      unsigned int p_int = float_to_uint(p_sat, P_MIN, P_MAX, 16);
      unsigned int v_int = float_to_uint(v_sat, V_MIN, V_MAX, 12);
      unsigned int kp_int = float_to_uint(kp_sat, KP_MIN, KP_MAX, 12);
      unsigned int kd_int = float_to_uint(kd_sat, KD_MIN, KD_MAX, 12);
      unsigned int t_int = float_to_uint(t_sat, T_MIN, T_MAX, 12);

      struct can_frame out_frame;
      out_frame.can_id = frame->id;
      out_frame.can_dlc = 8;
      //pack ints into the can buffer
      out_frame.data[0] = p_int >> 8;
      out_frame.data[1] = p_int & 0xFF;
      out_frame.data[2] = v_int >> 4;
      out_frame.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
      out_frame.data[4] = kp_int & 0xFF;
      out_frame.data[5] = kd_int >> 4;
      out_frame.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
      out_frame.data[7] = t_int & 0xFF;
    
      //Send
      CAN_MUTEX.lock();
      mcp2515.sendMessage(&out_frame);
      CAN_MUTEX.unlock();
    }
    
    inline MCP2515::ERROR readCAN(motor_frame_t* frame) {
      /* Reads a CAN message with ID, POS, VEL, and TOR being put into frame */
      struct can_frame in_frame;
      CAN_MUTEX.lock();
      MCP2515::ERROR err = mcp2515.readMessage(&in_frame);
      CAN_MUTEX.unlock();
      if (err == MCP2515::ERROR_OK) {   
        unsigned int ID = in_frame.data[0];
        unsigned int p_int = (in_frame.data[1] << 8) | in_frame.data[2];
        unsigned int v_int = (in_frame.data[3] << 4) | (in_frame.data[4] >> 4);
        unsigned int i_int = ((in_frame.data[4] & 0xF) << 8) | in_frame.data[5];

        frame->id = ID;
        frame->pos = uint_to_float(p_int, P_MIN, P_MAX, 16);
        frame->vel = uint_to_float(v_int, V_MIN, V_MAX, 12);
        frame->tor = uint_to_float(i_int, -T_MAX, T_MAX, 12);
             
      } else if(err != MCP2515::ERROR_NOMSG) {
        //Serial.print("Read Error: ");
        //Serial.println(err);
      }
    }
    
  private:
    MCP2515 mcp2515;
    float scaling_factor;
    static constexpr float DEF_TIMEOUT = 3;
    rtos::Mutex CAN_MUTEX;
    
  /* These functions will only output and send the correct values
     when the max and min values are properly defined above. */
  float float_to_uint(float x, float x_min, float x_max, int bits)  {
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12) {
      pgg = (unsigned int) ((x-offset)*4095.0/span); 
    }
    if (bits == 16) {
      pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
  }
  float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits == 12) {
      pgg = ((float)x_int)*span/4095.0 + offset;
    }
    if (bits == 16) {
      pgg = ((float)x_int)*span/65535.0 + offset;
    }
    return pgg;
  }
}; //End Class 

//Class instantiation in the Header. Never do this. This was done to interface with spaghetti code. 
akxMotor akMotor;

#endif
