#ifndef SYSTEM_HEADER
#define SYSTEM_HEADER
#include <Metro.h> // Include the Metro library
#include <SoftwareSerial.h>
#include "Parameters.hpp"
#include "Exoskeleton.hpp"

class Trial {
  int bluetoothStream = 0;
  Metro reportDataTimer = Metro(10);
  Metro receiverTimer = Metro(1);
};

struct System{
  Trial* trial = new Trial;
  SoftwareSerial bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
  // ===== A Exo =====

  elapsedMillis timeElapsed;
  int streamTimerCount = 0;
};
#endif
