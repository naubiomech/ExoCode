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
#define A1 1
#define A2 2
#define A3 3
#define A0 0
#define A5 5
#define A6 6
#define A12 12
#define A13 13
#define A14 14
#define A15 15

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
