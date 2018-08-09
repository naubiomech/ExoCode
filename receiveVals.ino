#include "receiveVals.h"

void receiveVals(int bytesExpected)
{
  int k = 0;
  while ((k < bytesExpected))
  {
    while (bluetooth.available() > 0)
    {
      int fromBluetooth = bluetooth.read();
      holdon[k] = fromBluetooth;               //Store all recieved bytes in subsequent memory spaces
      k = k + 1;                                  //Increments memory space
    }
  }
}

