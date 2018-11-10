#ifndef SYSTEM_HEADER
#define SYSTEM_HEADER
#include <Metro.h> // Include the Metro library
#include <SoftwareSerial.h>
#include "Parameters.hpp"
#include "Exoskeleton.hpp"
#include "Pins.hpp"

class Trial {
  int bluetoothStream = 0;
  Metro reportDataTimer = Metro(10);
  Metro receiverTimer = Metro(1);
};

class ExoSystem{
public:
  ExoSystem(ExoPins* exoPins);
  Trial* trial = new Trial();
  SoftwareSerial* bluetooth;
  elapsedMillis timeElapsed;
  int streamTimerCount = 0;
};
#endif
