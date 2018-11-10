#ifndef SYSTEM_HEADER
#define SYSTEM_HEADER
#include <Metro.h> // Include the Metro library
#include <SoftwareSerial.h>
#include "Parameters.hpp"
#include "Exoskeleton.hpp"
#include "Pins.hpp"
#include <Metro.h>

class Trial {
public:
  int bluetoothStream = 0;
  Metro reportDataTimer = Metro(10);
  Metro receiveDataTimer = Metro(1);
};

class ExoSystem{
public:
  ExoSystem(ExoPins* exoPins);
  Trial* trial = new Trial();
  SoftwareSerial* commandSerial;
  elapsedMillis timeElapsed;
  int streamTimerCount = 0;
  void startTrial();
  void endTrial();
  Exoskeleton* exo;
};
#endif
