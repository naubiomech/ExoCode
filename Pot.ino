/*Potentiometer code
 * This code is very basic but has been kept as an outside function for clarity. Here is how the potentiometer is used: calculate the Vo or resistance (doesn't matter) and you 
 * can see distance change over time (velocity) for checking if we are in early or late swing and you can use the Vo value to get angle of the leg. Must stabilize the signal 
 * however with a decoupling capacitor.
 * 
 */
/*
const unsigned int pot_sense_LL = A4;
double pot_LL;

const unsigned int pot_sense_RL = A11;
double pot_RL;
*/

double pot(const unsigned int pin) {
  double Vo;
  //Serial.println(analogRead(pin));
  //using voltage divider: 3.3 V -- >potR1<-- Vo -- >potR2< -- ground
  //Vo from analog read: 3.3* analog read/ max read (4096) || I = Vo/R || FSR resistance = (3.3V - Vo)/I 
  Vo = 3.3 * analogRead(pin)/4096;

  Vo = (Vo*300)/3.3;
  
  return Vo; 
}

