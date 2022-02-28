/*
 * Class to set an RGB LED to different colors based on the state of the system
 * 
 * Constructor: Status_Led(int rPin, int gPin, int bPin) or (int rPin, int gPin, int bPin, int brightness)
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

Status_Led::Status_Led(int rPin, int gPin, int bPin)
{
  // See header file for information on what each variable is for.
  _rPin = rPin;
  _gPin = gPin;
  _bPin = bPin;
  
  _brightness = 125; // range 0 - 255, off to full on.
  
  _currentMessage = STATUS_MESSAGE_TRIAL_OFF;  // initalize message to trial off
  
  // Configure the pin for the LED
  pinMode(_rPin, OUTPUT);  // sets the pin as output
  pinMode(_gPin, OUTPUT);  // sets the pin as output
  pinMode(_bPin, OUTPUT);  // sets the pin as output
  
  updateLed(_currentMessage); // set status
  
}

Status_Led::Status_Led(int rPin, int gPin, int bPin, int brightness)
{
  // See header file for information on what each variable is for.
  _rPin = rPin;
  _gPin = gPin;
  _bPin = bPin;
  
  _brightness = brightness ; // range 0 - 255, off to full on.
  
  _currentMessage = STATUS_MESSAGE_TRIAL_OFF;  // initalize message to trial off
  
  // Configure the pin for the LED
  pinMode(_rPin, OUTPUT);  // sets the pin as output
  pinMode(_gPin, OUTPUT);  // sets the pin as output
  pinMode(_bPin, OUTPUT);  // sets the pin as output
  
  updateLed(_currentMessage); // set status

}

/*
Public
*/

/*
* Change the message and LED state
*/
void Status_Led::updateLed(int message)
{
  _currentMessage = message;  // Update _currentMessage
  _setColor(_messageColors[_currentMessage][0],_messageColors[_currentMessage][1],_messageColors[_currentMessage][2]);   // Set the LED state
}


/*
 * Change the brightness.  Only used when NO_PWM is false.
 */
void Status_Led::setBrightness(int brightness)
{
  _brightness = brightness;
}



/*
Protected
*/

/*
 * Set LED state based on color values
 */
void Status_Led::_setColor(int rColor, int gColor, int bColor)
{
  if (NO_PWM)  // using simple digital pins
  {
    digitalWrite(_rPin, (STATUS_LED_OFF_STATE == 0) ? rColor >= 127 : rColor < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
    digitalWrite(_gPin, (STATUS_LED_OFF_STATE == 0) ? gColor >= 127 : gColor < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
    digitalWrite(_bPin, (STATUS_LED_OFF_STATE == 0) ? bColor >= 127 : bColor < 127);  // If the off state is low, LED turns on if color is >127 (1 on, 0 off), else the LED turns on if the >127 but the state is switched (0 on, 1 off)
  }
  else
  {
    int rColorScaled = floor(rColor * _brightness/255); // scale by brightness
    int gColorScaled = floor(gColor * _brightness/255); // scale by brightness
    int bColorScaled = floor(bColor * _brightness/255); // scale by brightness
    
    analogWrite(_rPin, abs(STATUS_LED_OFF_STATE - rColorScaled)); // if 0 is the off state will set rColorScaled value, if 255 is off state will set 255 - rColorScaled effectively inverting the PWM signal
    analogWrite(_gPin, abs(STATUS_LED_OFF_STATE - gColorScaled)); // if 0 is the off state will set gColorScaled value, if 255 is off state will set 255 - gColorScaled effectively inverting the PWM signal
    analogWrite(_bPin, abs(STATUS_LED_OFF_STATE - bColorScaled)); // if 0 is the off state will set bColorScaled value, if 255 is off state will set 255 - bColorScaled effectively inverting the PWM signal
  }
}
