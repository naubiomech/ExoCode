#include "System.hpp"
#include "Pins.hpp"

ExoSystem::ExoSystem(ExoPins* exoPins){
  trial = new Trial();
  bluetooth = new SoftwareSerial(BLUETOOTH_TX_PIN, BLUETOTH_RX_PIN);
  bluetooth->begin(115200);

  exo = new Exoskeleton(exoPins);
}
