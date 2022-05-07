// Wave Generator

const double waveSpacing = 0.002; //Time step
const int waveLength = 2/waveSpacing; //Total number of time points. e.g., 2 seconds/0.002 seconds = 1000 data points
volatile double tWave = 0; //Time 
float *wave = new float(waveLength);

void calculateSinWave()
{
  wave[0] = 0;
  for (int i = 1; i < waveLength+1; i++) {
    tWave = tWave + waveSpacing;
    wave[i] = sin(2*pi*tWave);
  }
}
