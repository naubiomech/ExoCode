#include "Board.h"

void setupBoard(){
  // enable bluetooth
  bluetooth.begin(115200);
  Serial.begin(115200);

  Serial.println("Starting");

  analogWriteResolution(12);                                          //change resolution to 12 bits
  analogReadResolution(12);                                           //ditto

  // The led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // set pin mode for left and right sides
  pinMode(MOTOR_ENABLE_PIN, OUTPUT); //Enable disable the motors
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

#ifdef IMU_BOARD
  bno = Adafruit_BNO055(WIRE_BUS, 1, BNO055_ADDRESS_A, I2C_MASTER, IMU_1_PINS, I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);
#endif
}
