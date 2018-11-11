#include "System.hpp"
#include "Pins.hpp"

ExoSystem::ExoSystem(ExoPins* exoPins){
  trial = new Trial();
  commandSerial = new SoftwareSerial(exoPins->bluetooth_tx, exoPins->bluetooth_rx);
  commandSerial->begin(115200);

  exo = new Exoskeleton(exoPins);
  report = exo->generateReport();
}

void ExoSystem::startTrial(){
  exo->enableExo();
  trial->bluetoothStream = 1;
  trial->reportDataTimer.reset();
  trial->receiveDataTimer.reset();
}

void ExoSystem::endTrial(){
  exo->disableExo();
  trial->bluetoothStream = 1;
}
