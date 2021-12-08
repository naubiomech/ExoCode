const double spacing = 0.002;
const int waveLength = 1/0.001;
double A = 7500.0/15000.0;
double wave[waveLength] = {};

void calculateStep()
{
  for (int i = 0; i <= waveLength; i++) {
    if (i <= waveLength/4) {
      wave[i] = A;
    } else if (i>waveLength/4 && i<=waveLength/2) {
      wave[i] = -A;
    } else {
      wave[i] = 0;
    }
  }
}
