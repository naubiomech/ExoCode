#ifndef ARDUINO_HEADER
#define ARDUINO_HEADER

#ifdef ARDUINO

#include <Arduino.h>
#include <i2c_t3.h>
#include <SoftwareSerial.h>
#include <Metro.h>
#include "PID_v2.h"
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

#else
#include <stdio.h>
#include <stdlib.h>

class SoftwareSerial{
public:
  SoftwareSerial(int, int);
  bool begin(int);
  void write(char character);
  void write(const char* str);
  void print(double);
  void print(const char[]);
  void println(const char[]);
  void println(double);

  int read();
  bool available();
};
#define REVERSE 1
#define AUTOMATIC 1

#define WIRE_BUS 1
#define I2C_MASTER 1
#define I2C_PULLUP_EXT 1
#define I2C_RATE_100 1
#define I2C_OP_MODE_ISR 1

enum i2c_pins {I2C_PINS_3_4 = 0,
               I2C_PINS_7_8,
               I2C_PINS_37_38};

typedef int uint8_t;
struct sensors_event_t{
  struct {
    double x;
    double y;
    double z;
  } orientation;
};

class Adafruit_BNO055{
public:
  Adafruit_BNO055(int, int, unsigned int, int, i2c_pins, int, int, int);
  bool begin();
  void getEvent(sensors_event_t*);
  void getCalibration(uint8_t*, uint8_t*, uint8_t*, uint8_t*);
  bool isFullyCalibrated();

};

class Metro{
public:
  Metro(unsigned long interval);
  void reset();
  bool check();
};

class PID{
public:
  PID(double*, double*, double*, double, double, double, int);
  void SetMode(int);
  void SetOutputLimits(int,int);
  void SetSampleTime(int);
  void Compute_KF(double);

};

#define INPUT 1
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define A1 101
#define A2 102
#define A3 103
#define A0 100
#define A5 105
#define A6 106
#define A12 112
#define A13 113
#define A14 114
#define A15 115
#define A16 116
#define A17 117

void delay(double);
double pow(double, double);
double exp(double);
double round(double);
double abs(double);
double max(double, double);
double min(double, double);
unsigned long millis();
void pinMode(int, int);
void analogReadResolution(int);
void analogWriteResolution(int);
int analogRead(int);
void analogWrite(int, int);
int digitalRead(int);
void digitalWrite(int, int);

extern SoftwareSerial Serial;
#endif

#endif
