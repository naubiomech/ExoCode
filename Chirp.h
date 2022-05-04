// Chirp Generator

const double chirpSpacing = 0.002; //Time step
const int timeLength = 30/chirpSpacing; //Total number of time points = (30 seconds)/spacing.
volatile double t = 0; //Time
float *chirp = new float(timeLength); //Sinusoid series

void calculateChirp()
{
  chirp[0]= 0;
  for (int i = 1; i < timeLength+1; i++) {
    t = t + chirpSpacing;
    chirp[i] = sin(PI*t*t); // Linear chirp signal x(t) = sin[2pi*((c/2)*t^2 + f0*t]. c = 1, f0 = 0. Up to 30 Hz in 30 seconds 
  }
}
