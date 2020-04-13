double spacing = 0.001;
double waveLength = 10001;
double x[10001] = {};
double A[10001] = {};
double wave[10001] = {};

void calculateWave()
{
  x[0] = 0;
  A[0] = 10;
  wave[0] = 0;
  for (int i = 1; i < waveLength; i++) {
    x[i + 1] = x[i] + spacing;
    A[i + 1] = 10*floor(x[i+1]) + 10;
    wave[i+1] = A[i + 1]*sin(8 * PI * x[i + 1]);
  }
  return;
}
