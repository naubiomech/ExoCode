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


#ifndef Status_Led_h
#define Status_Led_h

#include "Arduino.h"

// Define the on and off state of the LED.  This is handy for if you are using a P Channel MOSFET where low is on.
#define STATUS_LED_ON_STATE 0
#define STATUS_LED_OFF_STATE 255

// color assumes 255 is on.  The code will use the on/off state above to compensate for the code.
#define STATUS_MESSAGE_LED_OFF 0  // set the message index
#define STATUS_COLOR_LED_OFF {0, 0, 0}  // set the color in {R, G, B} format 0-255

#define STATUS_MESSAGE_TRIAL_OFF 1  // set the message index
#define STATUS_COLOR_TRIAL_OFF {0, 0, 255}   // set the color in {R, G, B} format 0-255 

#define STATUS_MESSAGE_TRIAL_ON 2 // set the message index
#define STATUS_COLOR_TRIAL_ON {0, 255, 0}  // set the color in {R, G, B} format 0-255

#define STATUS_MESSAGE_ERROR 3  // set the message index
#define STATUS_COLOR_ERROR {255, 0, 0}  // set the color in {R, G, B} format 0-255



#define NO_PWM true // true if using simple digital pins, false if using pwm pins


// Declare the class
class Status_Led
{
  public:
    // Constructors one you can set the default LED State
    Status_Led(int rPin, int gPin, int bPin);   // pins are the pins assocated with the different LED inputs
    Status_Led(int rPin, int gPin, int bPin, int brightness);  // pins are the pins assocated with the different LED inputs, brightness is used to scale the colors that are sent: color * brightness/255
   
    void updateLed(int message); // Changes the LED State to the current state
    void setBrightness(int brightness);  // Used if you need to change the brightness after initialization, brightness is used to scale the colors that are sent: color * brightness/255
    
  private:
  
    void _setColor(int R, int G, int B);  // changes the color R, G, and B are 0-255 values to set the corresponding colors.
  
    int _rPin;  // pin used for the red LED
    int _gPin;  // pin used for the green LED
    int _bPin;  // pin used for the blue LED
    int _brightness;  // Brightness of LED this scales the RGB colors, color * brightness/255
    int _currentMessage;  // index of the current message used to select the correct color.
    
    // make sure to keep in index order from defines, this is an array of the colors to use _messageColors[_currentMessage][color] where color is 0 for r, 1 for g, and 2 for b
    int _messageColors[4][3] = {  STATUS_COLOR_LED_OFF, \
                  STATUS_COLOR_TRIAL_OFF, \
                  STATUS_COLOR_TRIAL_ON, \
                  STATUS_COLOR_ERROR};
};

#endif
