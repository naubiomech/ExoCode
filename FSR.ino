/*FSR Code
 * This code is very basic but is kept as an outside function for clarity. The FSR readings are used to control state based actions based on the part of the gait cycle the patient
 * is in.
 */

/*
const unsigned int fsr_sense_LL = A2; //fsr analog sensor pin
double fsr_LL; // fsr value 
 
const unsigned int fsr_sense_RL = A10; //fsr analog sensor pin
double fsr_RL; // fsr value 
*/

double fsr(const unsigned int pin) {
  double Vo;
  analogRead(pin);
  //using voltage divider: 3.3 V -- >FSR<-- Vo -- >R< (1000) -- ground
  //Vo from analog read: 3.3* analog read/ max read (4096) || I = Vo/R || FSR resistance = (3.3V - Vo)/I 
  Vo = 10* 3.3 * analogRead(pin)/4096; //ZL Added in the 10* to scale the output
  
  //return (3.3 - Vo)*1000/Vo; //this is FSR resistance value
  return Vo;
}

