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
#include <math.h>

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
  
  _current_message = status_led_defs::messages::trial_off;  // initalize message to trial off
  
  // Configure the pin for the LED
  pinMode(_r_pin, OUTPUT);  // sets the pin as output
  pinMode(_g_pin, OUTPUT);  // sets the pin as output
  pinMode(_b_pin, OUTPUT);  // sets the pin as output
  
  update(_current_message); // set status
  
};

StatusLed::StatusLed(int r_pin, int g_pin, int b_pin, int brightness)
{
  // See header file for information on what each variable is for.
  _r_pin = r_pin;
  _g_pin = g_pin;
  _b_pin = b_pin;
  
  _brightness = brightness ; // range 0 - 255, off to full on.
  
  _current_message = status_led_defs::messages::trial_off;  // initalize message to trial off
  
  // Configure the pin for the LED
  pinMode(_r_pin, OUTPUT);  // sets the pin as output
  pinMode(_g_pin, OUTPUT);  // sets the pin as output
  pinMode(_b_pin, OUTPUT);  // sets the pin as output
  
  update(_current_message); // set status

};

/*
Public
*/

/*
* Change the message and LED state
*/
void StatusLed::update(uint8_t message)
{
    if (message != _current_message)
    {
        _current_message = message;  // Update _current_message
        _pattern_start_timestamp = millis();  // restart the timer
        _period_ms = _message_pattern[_current_message][1];
    }
    
    // int red = _message_colors[_current_message][0];
    // int green = _message_colors[_current_message][1];
    // int blue = _message_colors[_current_message][2];
        
    switch (_message_pattern[_current_message][0])
    {
        case status_led_defs::patterns::blink :
            _blink();
            break;    
        case status_led_defs::patterns::pulse :
            _pulse();
            break;
        case status_led_defs::patterns::rainbow :
            _rainbow_hsv();
            break;
        default : // solid
            _solid();
            break;
    }
      
      
    //_set_color(_message_colors[_current_message][0],_message_colors[_current_message][1],_message_colors[_current_message][2]);   // Set the LED state
};


/*
 * Change the brightness.  Only used when logic_micro_pins::status_has_pwm is true.
 */
void StatusLed::set_brightness(int brightness)
{
  _brightness = brightness;
};

/*
Protected
*/

/*
 * Set LED state based on color values
 */
void StatusLed::_set_color(int r_color, int g_color, int b_color)
{
    
  // Serial.print(r_color);
  // Serial.print("\t");
  // Serial.print(g_color);
  // Serial.print("\t");
  // Serial.print(b_color);
  // Serial.println();
  
  if (status_led_defs::has_pwm)  // using simple digital pins
  {
    int r_color_scaled = floor(r_color * _brightness/255); // scale by brightness
    int g_color_scaled = floor(g_color * _brightness/255); // scale by brightness
    int b_color_scaled = floor(b_color * _brightness/255); // scale by brightness
    
    analogWrite(_r_pin, abs(status_led_defs::off_state - r_color_scaled)); // if 0 is the off state will set r_colorScaled value, if 255 is off state will set 255 - r_colorScaled effectively inverting the PWM signal
    analogWrite(_g_pin, abs(status_led_defs::off_state - g_color_scaled)); // if 0 is the off state will set g_colorScaled value, if 255 is off state will set 255 - g_colorScaled effectively inverting the PWM signal
    analogWrite(_b_pin, abs(status_led_defs::off_state - b_color_scaled)); // if 0 is the off state will set b_colorScaled value, if 255 is off state will set 255 - b_colorScaled effectively inverting the PWM signal
    
  }
  else
  {
    digitalWrite(_r_pin, (status_led_defs::off_state == 0) ? r_color >= 127 : r_color < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
    digitalWrite(_g_pin, (status_led_defs::off_state == 0) ? g_color >= 127 : g_color < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
    digitalWrite(_b_pin, (status_led_defs::off_state == 0) ? b_color >= 127 : b_color < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
  }
};

/*
 * Displays whatever the current _message_colors values are continuously
 */
void StatusLed::_solid()
{
    _set_color(_message_colors[_current_message][0],_message_colors[_current_message][1],_message_colors[_current_message][2]);   // Set the LED state
    return;
};

/*
 * Dims and brightens the LED
 */
void StatusLed::_pulse()
{
    if (status_led_defs::has_pwm)
    {
        int timestamp = millis();
        if (timestamp - _pattern_start_timestamp > _period_ms)
        {
            _pattern_start_timestamp = timestamp;
        }
        // int time_diff = (timestamp - _pattern_start_timestamp);
        // _pattern_brightness_percent =  time_diff < (_period_ms / 2) ? 100 * time_diff / (_period_ms / 2) : 100 * (_period_ms - time_diff) / (_period_ms / 2);
        
        float angle_deg = 360.0 * (timestamp - _pattern_start_timestamp) / _period_ms;
        _pattern_brightness_percent = 100 * sin (angle_deg * PI / 180);
        //Serial.println(angle_deg);
        
        
        _set_color(_pattern_brightness_percent * _message_colors[_current_message][0] / 100, _pattern_brightness_percent * _message_colors[_current_message][1]/100, _pattern_brightness_percent * _message_colors[_current_message][2]/100);   // Set the LED state
    }
    else
    {
        // the _pattern_start_timestamp will get overwritten in this which should be ok but may cause some weirdness as it is getting checked twice.
        _blink();
    }
    return;
};

/*
 * Blinks the LED with equal amounts.
 */
void StatusLed::_blink()
{
    int timestamp = millis();
    if (timestamp - _pattern_start_timestamp > _period_ms)
    {
        _pattern_start_timestamp = timestamp;
    }
    
    bool on = (timestamp - _pattern_start_timestamp) < (_period_ms/2);
    _set_color(on * _message_colors[_current_message][0], on * _message_colors[_current_message][1], on * _message_colors[_current_message][2]);   // Set the LED state
    return;
};

/*
 * Brightens and dims each color as a sin wave where each color is phase shifted.
 */
void StatusLed::_rainbow_sin()
{
    // reset the pattern if we have gone past the period.
    int timestamp = millis();
    if (timestamp - _pattern_start_timestamp > _period_ms)
    {
        _pattern_start_timestamp = timestamp;
    }
    // rgb colors that will be set later on
    uint8_t colors [] = {0, 0, 0}; 
    // angular value where each colors sin wave starts
    int start_angles_deg[] = {240, 0, 120};
    // get angle based on the time.
    float angle_deg = 360.0 * (timestamp - _pattern_start_timestamp) / _period_ms;
    
    for(int i = 0; i < 3; i++)
    {
       // find the positive angle from the start angle
       int angle_from_start_deg = angle_deg < start_angles_deg[i] ?  angle_deg + 360 - start_angles_deg[i] : angle_deg - start_angles_deg[i];     
       // get the value of the color at the current angle.
       colors[i] = 255 * sin (angle_from_start_deg * PI / 180);
    }
    // Set the LED state
    _set_color(colors[0], colors[1], colors[2]);   
    return;
};

/*
 * Brightens and dims each color as a trapezoidal wave where each color is phase shifted.
 */
void StatusLed::_rainbow_hsv()
{
    int timestamp = millis();
    if (timestamp - _pattern_start_timestamp > _period_ms)
    {
        _pattern_start_timestamp = timestamp;
    }
    
    uint8_t colors [] = {0, 0, 0}; // rgb
    
    // color counts up for 60 degrees from the start.  Holds high for 120, then counts down for 60.
    int start_angles_deg[] = {240, 0, 120};
        
    float angle_deg = 360.0 * (timestamp - _pattern_start_timestamp) / _period_ms;
    int angle_from_start_deg = 0;
    
    
    for(int i = 0; i < 3; i++)
    {
       // find the positive angle from the start angle
       angle_from_start_deg = angle_deg < start_angles_deg[i] ? 360 + angle_deg - start_angles_deg[i] : angle_deg - start_angles_deg[i]; 
       if (angle_from_start_deg < 60)
       {
           colors[i] = 255 * angle_from_start_deg / 60;
       }
       else if (angle_from_start_deg < 180)
       {
           colors[i] = 255;
       }
       else if (angle_from_start_deg < 240)
       {
           colors[i] = 255 * (240 - angle_from_start_deg) / 60;
       }
       else 
       {
           colors[i] = 0;
       }
    }
    
    _set_color(colors[0], colors[1], colors[2]);   // Set the LED state
    return;
};