#ifndef EXO_BUILDER_HEADER
#define EXO_BUILDER_HEADER
#include "Port.hpp"
#include "Exoskeleton.hpp"

class LegBuilder;

class ExoBuilder{
private:
  RxPort* rx;
  TxPort* tx;
  LegBuilder* right_builder;
  LegBuilder* left_builder;
public:
  ExoBuilder* addTransceiver(TxPort* tx, RxPort* rx);
  LegBuilder* beginRightLeg();
  LegBuilder* beginLeftLeg();
  Exoskeleton* build();
};

#endif
