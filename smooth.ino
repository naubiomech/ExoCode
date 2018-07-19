double smooth(double data, double filterVal, double smoothedVal){
  if (filterVal > 1)
  {                 // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0)
  {
    filterVal = 0;
  }
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return (double)smoothedVal;
}
