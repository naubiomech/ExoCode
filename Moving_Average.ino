double Moving_Average(double* p_ref, int dim) {
  
  double count=0;

  for (int i = 0; i < dim; i++) {
    count=*(p_ref+i);
  }
  count=count/dim;
  return count;
}
