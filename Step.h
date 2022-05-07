// Step Generator

const double stepSpacing = 0.002;
const int stepLength = 10/stepSpacing;
double A = 1;
float *stp = new float(stepLength);

void calculateStepBi()
// Bidirectional step function
{
  for (int i = 0; i < stepLength; i++) {
    if (i <= stepLength/4) {
      stp[i] = A;
    } else if (i>stepLength/4 && i<=stepLength/2) {
      stp[i] = -A;
    } else {
      stp[i] = 0;
    }
  }
}

void calculateStepUni()
// Unidirectional step function
{
  for (int i = 0; i < stepLength; i++) {
    if (i <= stepLength/2) {
      stp[i] = A;
    } else {
      stp[i] = 0;
    }
  }
}
