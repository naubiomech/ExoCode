// Chirp Generator

const double chirpSpacing = 0.002; //Time step
const int dutyLength = 20/chirpSpacing; //Total number of time points = (30 seconds)/spacing.
const int timeLength = dutyLength*1.5;
volatile double t = 0; //Time
float *chirp = new float(timeLength); //Array for storing desired chirp profile
//float *chirpFreq = new float(timeLength); //Array for storing the chirp frequency

void calculateChirp()
{
  chirp[0]= 0;
 // chirpFreq[0] = 0;
  for (int i = 1; i < dutyLength+1; i++) {
    t = t + chirpSpacing;
    chirp[i] = sin(0.5*PI*t*t); // Linear chirp signal x(t) = sin[2pi*((c/2)*t^2 + f0*t]. c = 0.5, f0 = 0. Up to 30 Hz in 30 seconds 
    //chirpFreq[i] = t; // The current sine wave instantaneous frequency
  }
  for (int i = dutyLength; i < timeLength+1; i++) {
    chirp[i] = 0; //Fill with zeroes for the rest of the time
  }
}
