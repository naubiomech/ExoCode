//This function is written with the intent of differentiating the right leg from the left leg.
//The idea is that we use a wire to short the input pin to either ground or 3.3v, and a digitalRead can tell the diference between the legs
//      LEFT LEG READS HIGH
//      RIGHT LEG READS LOW
double whichLeg(const unsigned int pin, double Setpoint_Ankle) {
  if (digitalRead(pin) == HIGH)
  {
    Setpoint_Ankle = abs(Setpoint_Ankle);
    //Serial.println("Check1");
  }
  if (digitalRead(pin) == LOW)
  {
    Setpoint_Ankle = -abs(Setpoint_Ankle);
    //Serial.println("Check2");
  }
  return Setpoint_Ankle;
}

