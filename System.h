#ifndef SYSTEM_HEADER
#define SYSTEM_HEADER
#include <Metro.h> // Include the Metro library

// ===== A Exo =====
Metro slowThisDown = Metro(1);  // Set the function to be called at no faster a rate than once per millisecond
Metro BnoControl = Metro(10);

//To interrupt and to schedule we take advantage of the
elapsedMillis timeElapsed;
double startTime = 0;
int streamTimerCount = 0;

int stream = 0;

char holdon[24];
char *holdOnPoint = &holdon[0];
char Peek = 'a';

#endif
