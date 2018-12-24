#ifndef COMMUNICATIONS_HEADER
#define COMMUNICATIONS_HEADER
#include "Transceiver.hpp"

class Communications{
private:
  Transceiver* transceiver;
public:
  Communications(Transceiver* transceiver);
  ~Communications();
  ExoMessage* receiveMessages(ExoReport* report);
  void sendReport(ExoReport* report);
};

#endif
