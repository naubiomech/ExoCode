#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER

#if BOARD_VERSION == AK_Board_V0_1

	#include <SPI.h>
	#include <Wire.h>
	// !!! CHANCE DOUBLE CHECK THE LIBS AND PINS.

	// I2C Pins
	const unsigned int SDA = A4;
	const unsigned int SCL = A5;
	
	// SPI Follower Pins
	const unsigned int MISO_PIN = D12;
	const unsigned int MOSI_PIN = D11;
	const unsigned int SCK_PIN = D13;
	const unsigned int CS_PIN = D10;
	const unsigned int SPI_MODE = 16;
  
#endif

#endif