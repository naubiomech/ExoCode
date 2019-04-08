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
  char readStr[25] = {70,11,0,0,0,0,0,0,0,0,0,0,0,0,0,-16,63,0,0,0,0,0,0,-16,63};
  Serial.setReadString(readStr, 25);
  MatlabTransceiver* trans = new MatlabTransceiver(&Serial);
  Communications* comms = new Communications(trans);
  ExoMessage* msg = comms->receiveMessages(NULL);
  delete comms;
  delete msg;
}

int main(){
  testExo();
  testComms();
}
#endif
#endif
