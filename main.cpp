#ifndef ARDUINO
#ifndef ROS
#include "Arduino.hpp"
#include "ExoBuilder.hpp"
#include "BoardBuilder.hpp"
#include "Port.hpp"
#include "Linked_List.hpp"
#include "FSR.hpp"
#include "Message.hpp"

Exoskeleton* setupSystem(){
  Serial.begin(115200);
  delay(500);
  Board* board = QuadBoardDirector().build();
  Serial.println("Beginning exo building..");
  Exoskeleton* exo = QuadExoDirector().build(board);
  Serial.println("Finished exo building");
  delete board;
  return exo;
}

void testExo(){
  Exoskeleton* exo = setupSystem();
  ExoMessageBuilder builder;
  builder.addPreCommand(new StartTrialCommand());
  ExoMessage* msg = builder.build();
  exo->processMessage(msg);
  delete msg;
  exo->receiveMessages();
  exo->run();
  exo->sendReport();
  delete exo;
}

void testComms(){
  const int sizeSet = 49;
  char readStrSet[sizeSet] = {77, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, 63,
                              0, 0, 0, 0, 0, 0, 8, 64, 0, 0, 0, 0, 0, 0, -16, 63, 0,
                              0, 0, 0, 0, 0, -16, 63, 0, 0, 0, 0, 0, 0, -16, 63};
  Serial.setReadString(readStrSet, sizeSet);
  Exoskeleton* exo = setupSystem();
  MatlabTransceiver* trans = new MatlabTransceiver(&Serial);
  Communications* comms = new Communications(trans);
  ExoMessage* msg = comms->receiveMessages(NULL);
  exo->processMessage(msg);
  delete msg;

  const int sizeGet = 25;
  char readStrGet[sizeGet] = {75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, -16, 63, 0, 0, 0, 0, 0, 0, 8, 64};
  Serial.setReadString(readStrGet, sizeGet);
  ExoReport* report = exo->generateReport();
  msg = comms->receiveMessages(report);
  delete msg;
  delete comms;
}

int main(){
	testExo();
	testComms();
}
#endif
#endif
