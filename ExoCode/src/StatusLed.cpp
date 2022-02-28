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

#include "Arduino.h"
#include "StatusLed.h"

/*
Constructors
*/

StatusLed::StatusLed(int r_pin, int g_pin, int b_pin)
{
  // See header file for information on what each variable is for.
  _r_pin = r_pin;
  _g_pin = g_pin;
  _b_pin = b_pin;
  
  _brightness = 125; // range 0 - 255, off to full on.
  
  _current_message = STATUS_MESSAGE_TRIAL_OFF;  // initalize message to trial off
  
  // Configure the pin for the LED
  pinMode(_r_pin, OUTPUT);  // sets the pin as output
  pinMode(_g_pin, OUTPUT);  // sets the pin as output
  pinMode(_b_pin, OUTPUT);  // sets the pin as output
  
  update_led(_current_message); // set status
  
}

StatusLed::StatusLed(int r_pin, int g_pin, int b_pin, int brightness)
{
  // See header file for information on what each variable is for.
  _r_pin = r_pin;
  _g_pin = g_pin;
  _b_pin = b_pin;
  
  _brightness = brightness ; // range 0 - 255, off to full on.
  
  _current_message = STATUS_MESSAGE_TRIAL_OFF;  // initalize message to trial off
  
  // Configure the pin for the LED
  pinMode(_r_pin, OUTPUT);  // sets the pin as output
  pinMode(_g_pin, OUTPUT);  // sets the pin as output
  pinMode(_b_pin, OUTPUT);  // sets the pin as output
  
  update_led(_current_message); // set status

}

/*
Public
*/

/*
* Change the message and LED state
*/
void StatusLed::update_led(int message)
{
  _current_message = message;  // Update _current_message
  _set_color(_message_colors[_current_message][0],_message_colors[_current_message][1],_message_colors[_current_message][2]);   // Set the LED state
}


/*
 * Change the brightness.  Only used when NO_PWM is false.
 */
void StatusLed::set_brightness(int brightness)
{
  _brightness = brightness;
}



/*
Protected
*/

/*
 * Set LED state based on color values
 */
void StatusLed::_set_color(int r_color, int g_color, int b_color)
{
  if (NO_PWM)  // using simple digital pins
  {
    digitalWrite(_r_pin, (STATUS_LED_OFF_STATE == 0) ? r_color >= 127 : r_color < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
    digitalWrite(_g_pin, (STATUS_LED_OFF_STATE == 0) ? g_color >= 127 : g_color < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
    digitalWrite(_b_pin, (STATUS_LED_OFF_STATE == 0) ? b_color >= 127 : b_color < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
  }
  else
  {
    int r_color_scaled = floor(r_color * _brightness/255); // scale by brightness
    int g_color_scaled = floor(g_color * _brightness/255); // scale by brightness
    int b_color_scaled = floor(b_color * _brightness/255); // scale by brightness
    
    analogWrite(_r_pin, abs(STATUS_LED_OFF_STATE - r_color_scaled)); // if 0 is the off state will set r_colorScaled value, if 255 is off state will set 255 - r_colorScaled effectively inverting the PWM signal
    analogWrite(_g_pin, abs(STATUS_LED_OFF_STATE - g_color_scaled)); // if 0 is the off state will set g_colorScaled value, if 255 is off state will set 255 - g_colorScaled effectively inverting the PWM signal
    analogWrite(_b_pin, abs(STATUS_LED_OFF_STATE - b_color_scaled)); // if 0 is the off state will set b_colorScaled value, if 255 is off state will set 255 - b_colorScaled effectively inverting the PWM signal
  }
}
