#include "Communications.hpp"

Communications::Communications(Transceiver* transceiver){
  this->transceiver = transceiver;
}

Communications::~Communications(){
  delete transceiver;
}

ExoMessage* Communications::receiveMessages(ExoReport* report){
  return transceiver->receiveMessages(report);
}

void Communications::sendReport(ExoReport* report){
  transceiver->sendReport(report);
}
