#include "Arduino.hpp"
#ifndef ARDUINO
#include <stdio.h>
#include <sys/time.h>

SoftwareSerial Serial = SoftwareSerial(0,0);

SoftwareSerial::SoftwareSerial(int a, int b){}
bool SoftwareSerial::begin(int a){return true;}
void SoftwareSerial::write(char str){
  printf("%c", str);
}
void SoftwareSerial::write(const char* str){
  printf("%s", str);
}

void SoftwareSerial::print(const char* str){
  printf("%s", str);
}

void SoftwareSerial::print(double val){
  printf("%lf", val);
}

void SoftwareSerial::println(const char* str){
  printf("%s\n", str);
}

void SoftwareSerial::println(double val){
  printf("%lf\n", val);
}

int SoftwareSerial::read(){
  return -1;
}

bool SoftwareSerial::available(){return false;};


Adafruit_BNO055::Adafruit_BNO055(int, int, unsigned int, int, i2c_pins, int, int, int){}
bool Adafruit_BNO055::begin(){return true;}
void Adafruit_BNO055::getEvent(sensors_event_t*){}
void Adafruit_BNO055::getCalibration(uint8_t*, uint8_t*, uint8_t*, uint8_t*){}
bool Adafruit_BNO055::isFullyCalibrated(){return true;}


Metro::Metro(unsigned long interval){}
void Metro::reset(){}
bool Metro::check(){return true;}

PID::PID(double*, double*, double*, double, double, double, int){}
void PID::SetMode(int){}
void PID::SetOutputLimits(int,int){}
void PID::SetSampleTime(int){}
void PID::Compute_KF(double){}

void delay(double){}
double pow(double, double){return 0;}
double exp(double){return 0;}
double round(double){return 0;}
double abs(double){return 0;}
double max(double, double){return 0;}
double min(double, double){return 0;}
unsigned long int millis(){
  struct timeval tv;

  gettimeofday(&tv, NULL);

  unsigned long long millisecondsSinceEpoch =
    (unsigned long long)(tv.tv_sec) * 1000 +
    (unsigned long long)(tv.tv_usec) / 1000;

  return millisecondsSinceEpoch;
}
void pinMode(int, int){}
void analogReadResolution(int){}
void analogWriteResolution(int){}
int analogRead(int){return 0;}
void analogWrite(int, int){}
int digitalRead(int){return 0;}
void digitalWrite(int, int){}

#endif
