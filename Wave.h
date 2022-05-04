// Wave Generator

const double spacing = PI*0.005;
const int waveLength = 2*PI/spacing;
double x[waveLength] = {};
double A = 0.5;
double wave[waveLength] = {};

void calculateWave()
{
  x[0] = 0;
  wave[0] = 0;
  for (int i = 0; i < waveLength; i++) {
    x[i + 1] = x[i] + spacing;
    wave[i+1] = A*sin(x[i+1]);
  }
}
