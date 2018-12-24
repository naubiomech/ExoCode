#include "Communications.hpp"

Communications::Communications(Transceiver* transceiver){
  this->transceiver = transceiver;
}

Communications::~Communications(){
  delete transceiver;
}

ExoMessage* Communications::receiveMessages(ExoReport* report){
  ExoMessageBuilder builder;
  while(transceiver->dataAvailable()){
    receiveMessage(&builder, report);
  }
  return builder.build();
}

void Communications::processMessage(CommandCode code, ExoMessageBuilder* msg_builder, ExoReport* report){
  Transmission* transmission = transmission_creator->create(transceiver, code);
  transmission->process(msg_builder, report);
  delete transmission;
}

void Communications::receiveMessage(ExoMessageBuilder* msg_builder, ExoReport* report){

  if (transceiver->noDataAvailable()) {
    return;
  }

  transceiver->receiveHeader();
  CommandCode code = transceiver->receiveCommand();
  processMessage(code, msg_builder, report);

}

void Communications::sendReport(ExoReport* report){
  ExoMessageBuilder builder;
  processMessage('?', &builder, report);
}
