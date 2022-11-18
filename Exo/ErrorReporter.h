/*
 *  This class reports errors over bluetooth. 
 */

#ifndef ERRORREPORTER_H
#define ERRORREPORTER_H
#include "Msg_functions.h"

typedef enum {
  TRACKING,
  SAT_PID,
  TRQ_THRSH
} ERRORS;

typedef enum {
  LEFT,
  RIGHT
} LEGS;

class ErrorReporter {
  public:
  void report(ERRORS error, LEGS leg) {
    send_error = (10*(static_cast<int>(leg)))+(static_cast<int>(error));
  }
};

#endif
