double pot(const unsigned int pin_l) {
    // Even if it is odd I prefer to keep all the ratio
  // for the conversions from bit to V from V to deg.
  return 3.3 * analogRead(pin_l) / 4096 * 300 / 3.3; 
}

