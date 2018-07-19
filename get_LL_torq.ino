double get_LL_torq() 
{                                               //flexion is positive 8.10.16, gets the torque of the right leg 
  double Torq = 56.5/(2.1)*(analogRead(A4)*(3.3/4096)-Tcal_LL);
  return Torq;             //neg is here for right leg, returns the torque value of the right leg (Newton-Meters)
}
