#include "System.hpp"
#include "Pins.hpp"
#include "Receive_and_Transmit.hpp"

ExoSystem::ExoSystem(ExoPins* exoPins){
  trial = new Trial();
  commandSerial = new SoftwareSerial(exoPins->bluetooth_tx, exoPins->bluetooth_rx);
  commandSerial->begin(115200);

  exo = new Exoskeleton(exoPins);
  report = exo->generateReport();
}

void ExoSystem::run(){
  if (this->trial->bluetoothStream == 1) {
    if (this->trial->reportDataTimer.check()) {
      send_report(this);
      this->trial->reportDataTimer.reset();
    }
  }
  exo->run();
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
