/*
 * Class to set an RGB LED to different colors based on the state of the system
 * 
 * Constructor: StatusLed(int r_pin, int g_pin, int b_pin) or (int r_pin, int g_pin, int b_pin, int brightness)
 *   The pins are the RGB LED pins ideally they are PWM but can also handle simple digital pins.
 *      In the header set NO_PWM to true or false depending on if you have PWM or simple digital pins.
 *   Brightness sets the brightness from 255 to 0, this is ignored for simple digital pins
 *   
 * updateLed(int message) method sets the color of the LED, these messages can be found in the preprocessor part of the header.
 * setBrightness(int brightness) method is used to change the brightness after initialization.
 * 
 * P. Stegall Dec. 2021
*/


#ifndef StatusLed_h
#define StatusLed_h

#include "Arduino.h"
#include "Board.h"

// Define the on and off state of the LED.  This is handy for if you are using a P Channel MOSFET where low is on.
//#define STATUS_LED_ON_STATE 0
//#define STATUS_LED_OFF_STATE 255

// color assumes 255 is on.  The code will use the on/off state above to compensate for the code.
//#define STATUS_MESSAGE_LED_OFF 0  // set the message index
//#define STATUS_COLOR_LED_OFF {0, 0, 0}  // set the color in {R, G, B} format 0-255

//#define STATUS_MESSAGE_TRIAL_OFF 1  // set the message index
//#define STATUS_COLOR_TRIAL_OFF {0, 0, 255}   // set the color in {R, G, B} format 0-255 

//#define STATUS_MESSAGE_TRIAL_ON 2 // set the message index
//#define STATUS_COLOR_TRIAL_ON {0, 255, 0}  // set the color in {R, G, B} format 0-255

//#define STATUS_MESSAGE_ERROR 3  // set the message index
//#define STATUS_COLOR_ERROR {255, 0, 0}  // set the color in {R, G, B} format 0-255

//#define NO_PWM true // true if using simple digital pins, false if using pwm pins


namespace status_led_defs
{
    namespace messages 
    {
        
        //0b error ,trial_on, trial_off
        const uint8_t off =  0;
        const uint8_t trial_off = 1;
        const uint8_t trial_on = 2;
        const uint8_t test = 3; // generally won't be used.
        // Specific error messages will use the 5 highest bits, giving 31 error messages
        const uint8_t error = 0b00000100;
        
        const uint8_t error_left_heel_fsr =  1<<3 | error;
        const uint8_t error_left_toe =  2<<3 | error;
        const uint8_t error_right_heel_fsr =  3<<3 | error;
        const uint8_t error_right_toe_fsr =  4<<3 | error;
        
        const uint8_t error_left_hip_torque_sensor =  5<<3 | error;
        const uint8_t error_left_knee_torque_sensor =  6<<3 | error;
        const uint8_t error_left_ankle_torque_sensor =  7<<3 | error;
        const uint8_t error_right_hip_torque_sensor =  8<<3 | error;
        const uint8_t error_right_knee_torque_sensor =  9<<3 | error;
        const uint8_t error_right_ankle_torque_sensor =  10<<3 | error;
        
        const uint8_t error_left_hip_motor =  11<<3 | error;
        const uint8_t error_left_knee_motor =  12<<3 | error;
        const uint8_t error_left_ankle_motor =  13<<3 | error;
        const uint8_t error_right_hip_motor =  14<<3 | error;
        const uint8_t error_right_knee_motor =  15<<3 | error;
        const uint8_t error_right_ankle_motor =  16<<3 | error;
        
        const uint8_t error_left_hip_controller =  17<<3 | error;
        const uint8_t error_left_knee_controller =  18<<3 | error;
        const uint8_t error_left_ankle_controller =  19<<3 | error;
        const uint8_t error_right_hip_controller =  20<<3 | error;
        const uint8_t error_right_knee_controller =  21<<3 | error;
        const uint8_t error_right_ankle_controller =  22<<3 | error;
        
        
    }
    
    namespace colors // just used namespace due to the complex structure.
    {
        const int off[] =  {0, 0, 0};
        const int trial_off[] = {0, 0, 255};
        const int trial_on[] = {0, 255, 0};
        const int test[] = {0, 255, 0};
        const int error[] = {255, 0, 0};
        
    }
    
    namespace patterns // just used namespace due to the complex structure.
    {
        const uint8_t solid = 0;
        const uint8_t blink = 1;
        const uint8_t pulse = 2;
        const uint8_t rainbow = 3;
        
        
        //Format is pattern number and period in ms
        const int off[] =  {solid, 0}; // Solid
        const int trial_off[] = {solid,0}; // Solid
        const int trial_on[] = {pulse, 500}; // Solid
        const int test[] = {rainbow, 4000}; // pulse
        const int error[] = {blink, 250}; // blinking
        
    }
    
    const uint8_t on_state = logic_micro_pins::status_led_on_state;
    const uint8_t off_state = logic_micro_pins::status_led_off_state;  
    
    const bool has_pwm = logic_micro_pins::status_has_pwm;
    
}


// Declare the class
class StatusLed
{
  public:
    // Constructors one you can set the default LED State
    StatusLed(int r_pin, int g_pin, int b_pin);   // pins are the pins assocated with the different LED inputs
    StatusLed(int r_pin, int g_pin, int b_pin, int brightness);  // pins are the pins assocated with the different LED inputs, brightness is used to scale the colors that are sent: color * brightness/255
   
    void update(uint8_t message); // Changes the LED State to the current state
    void set_brightness(int brightness);  // Used if you need to change the brightness after initialization, brightness is used to scale the colors that are sent: color * brightness/255
    
  private:
  
    void _set_color(int R, int G, int B);  // changes the color R, G, and B are 0-255 values to set the corresponding colors.
    void _solid();
    void _pulse();
    void _blink();
    void _rainbow_sin();
    void _rainbow_hsv();
    
    int _r_pin;  // pin used for the red LED
    int _g_pin;  // pin used for the green LED
    int _b_pin;  // pin used for the blue LED
    int _brightness;  // Max brightness of LED this scales the RGB colors, color * brightness/255
    int _current_message;  // index of the current message used to select the correct color.
    
    int _pattern_start_timestamp; // keeps track of the time the current pattern has run.
    int _period_ms; // period of the pattern
    int _pattern_brightness_percent; // Percent of the  for the pattern brightness.
    
    // make sure to keep in index order from messages, this is an array of the colors to use _messageColors[_currentMessage][color] where color is 0 for r, 1 for g, and 2 for b.
    // This method of accessing array elements is bulky but works.
    const int _message_colors[5][3] = {{status_led_defs::colors::off[0], status_led_defs::colors::off[1], status_led_defs::colors::off[2]}, \
                {status_led_defs::colors::trial_off[0], status_led_defs::colors::trial_off[1], status_led_defs::colors::trial_off[2]}, \
                {status_led_defs::colors::trial_on[0], status_led_defs::colors::trial_on[1], status_led_defs::colors::trial_on[2]}, \
                {status_led_defs::colors::test[0], status_led_defs::colors::test[1], status_led_defs::colors::test[2]}, \
                {status_led_defs::colors::error[0], status_led_defs::colors::error[1], status_led_defs::colors::error[2]} \
                };
    
    // make sure to keep in index order from messages, this is an array of the colors to use _messageColors[_currentMessage][color] where color is 0 for r, 1 for g, and 2 for b.
    // This method of accessing array elements is bulky but works.
    const int _message_pattern[5][2] = {{status_led_defs::patterns::off[0], status_led_defs::patterns::off[1]}, \
                {status_led_defs::patterns::trial_off[0], status_led_defs::patterns::trial_off[1]}, \
                {status_led_defs::patterns::trial_on[0], status_led_defs::patterns::trial_on[1]}, \
                {status_led_defs::patterns::test[0], status_led_defs::patterns::test[1]}, \
                {status_led_defs::patterns::error[0], status_led_defs::patterns::error[1]} \
                };
              
    
};

#endif