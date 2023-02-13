/**
 * @file AngleSensorCode.ino
 * @author Chance Cuddeback
 * @brief This code uses the Nano 33 BLE Sense to read the angle of two AS5600 magnetic angle sensors. 
 * The AS5600 does not have a configurable I2C address, so the code uses two Software I2C buses to 
 * read from the sensors. The Hardware I2C bus is used as a slave to send data to a teensy. 
 * @date 2023-02-12
 * 
 */

#include <Arduino.h>
#include <Wire.h>
#include <utility>
#include <SoftwareI2C.h>  // https://github.com/micooke/SoftwareI2C
#include "utils.h"

// =============================== Helpers ===============================
#define LOG_LEVEL 3 // 0 = none, 1 = errors, 2 = warnings, 3 = debug
typedef enum {
    LOG_NONE = 0,
    LOG_ERROR = 1,
    LOG_WARN = 2,
    LOG_DEBUG = 3, 
    LOG_DISABLE,
} app_log_level_t;

/**
 * @brief Prints a message to the serial port if the log level is high enough
 * 
 * @param msg Message to print
 * @param level log level
 */

void logger(String msg, app_log_level_t level) {
    if (level <= LOG_LEVEL) {
        Serial.print(msg);
    }
}

/**
 * @brief Starts the serial port for logging
 * 
 * @param baud_rate 
 */
void start_logger(int baud_rate = 115200) {
    Serial.begin(baud_rate);
    delay(100);
    if (LOG_LEVEL == LOG_DEBUG) {
      while(!Serial);
    }
    logger("\nStarted logger", LOG_DEBUG);
}

/**
 * @brief Prints a message to the serial port and blinks the LED, spins forever
 * 
 * @param msg 
 */
inline static void app_error(String msg) {
    while (1) {
        logger(msg, LOG_ERROR);
        led_helper(1, 0, 0);
        delay(500);
        led_helper(0, 0, 0);
        delay(500);
    }
}

static const int led_pins[] = {22, 23, 24};
/**
 * @brief Writes RGB values to the LED
 * 
 * @param r Red 0-255
 * @param g Green 0-255
 * @param b Blue 0-255
 */
inline static void led_helper(bool r, bool g, bool b) {
  // LED is active low, must subtract by one
  digitalWrite(led_pins[0], !r);
  digitalWrite(led_pins[1], !g);
  digitalWrite(led_pins[2], !b);
};


// =============================== App Defines ===============================

#define SAMPLE_PERIOD 1000 // microseconds

// Address of the slave I2C bus, used to send data to the teensy
#define SLAVE_ADDRESS 0x04

// Define the Pins for the Software I2C bus
#define L_SDA_PIN 2
#define L_SCL_PIN 3
#define R_SDA_PIN 4
#define R_SCL_PIN 5

// Define the I2C addresses of the AS5600 sensor
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE 0x0E

// Assume 12 bit resolution
#define UINT_FLOAT_CONV 360/4096

// =============================== App Globals ===============================
// Declare Softwire pair
static std::pair<SoftwareI2C, SoftwareI2C> sWires = std::make_pair(
    SoftwareI2C(L_SDA_PIN, L_SCL_PIN), SoftwareI2C(R_SDA_PIN, R_SCL_PIN));

// Declare latest angle readings
static std::pair<float, float> angles = std::make_pair(0, 0);

// Last command received from the master
static volatile uint8_t working_cmd = 0x00;
static volatile uint8_t working_bytes = 0x00;

// =============================== Angle Functions ===============================
/**
 * @brief Get the angle data
 * 
 * @param sWire I2C bus
 * @return std::pair<uint8_t, uint8_t> low, high bytes
 */
std::pair<uint8_t, uint8_t> get_angle_data(SoftwareI2C sWire) {
    // Read the angle from the AS5600 sensor
    sWire.beginTransmission(AS5600_ADDRESS);
    sWire.write(AS5600_ANGLE);
    sWire.endTransmission();
    sWire.requestFrom(AS5600_ADDRESS, 2);
    uint8_t data[2];
    data[0] = sWire.read();
    data[1] = sWire.read();
    return std::make_pair(data[0], data[1]);
}
/**
 * @brief Get the angle object from the AS5600 sensor
 * 
 * @param sWire SoftwareI2C object
 * @return float angle, converted assuming 12 bit resolution
 */
float get_angle(SoftwareI2C sWire) {
    std::pair<uint8_t, uint8_t> data = get_angle_data(sWire);
    uint8_t data_u[2] = {data.first, data.second};
    float angle;
    utils::uint8_to_float(data_u, &angle);
    return angle;
}
/**
 * @brief Wrapper for get_angle
 * 
 * @return std::pair<float, float> Left and right angles, respectively
 */
std::pair<float, float> get_angles_pair() {
    // Read the angle from the AS5600 sensor
    return std::make_pair(get_angle(sWires.first), get_angle(sWires.second));
}


// =============================== I2C Functions ===============================
static const uint8_t ENABLE_CMD = 0x01;
static const uint8_t L_ANGLE_CMD = 0x02;
static const uint8_t R_ANGLE_CMD = 0x03;

inline static void on_receive(int bytes) {
    logger("\nOn Receive with: " + String(bytes), LOG_DEBUG);

    uint8_t cmd = Wire.read();
    logger("\nReceived: " + String(cmd), LOG_DEBUG);
    switch (cmd) {
        case ENABLE_CMD:
            logger("\nReceived command to turn on LED", LOG_DEBUG);
            led_helper(0, 1, 0);
            break;
        case L_ANGLE_CMD:
            logger("\nReceived command to get left angle", LOG_DEBUG);
            break;
        case R_ANGLE_CMD:
            logger("\nReceived command to get right angle", LOG_DEBUG);
            break;
        default:
            logger("\nReceived invalid command", LOG_WARN);
            return;
    }
    working_cmd = cmd;
    working_bytes = bytes;
}

inline static void on_request() {
    logger("\nOn Request with: " + String(working_cmd) + "bytes: " + String(working_bytes), LOG_DEBUG);

    if (working_cmd == 0) {
      logger("\nWorking command has not been initialized", LOG_WARN);
      return;
    }
    
    uint8_t bytes[2];
    switch (working_cmd) {
        case ENABLE_CMD:
            logger("\nSending handshake", LOG_DEBUG);
            Wire.write(0x01);
            break;
        case L_ANGLE_CMD:
            logger("\nSending left angle: " + String(angles.first), LOG_DEBUG);
            utils::float_to_uint8(angles.first, bytes);
            Wire.write(bytes[0]);
            Wire.write(bytes[1]);
            break;
        case R_ANGLE_CMD:
            logger("\nSending right angle: " + String(angles.second), LOG_DEBUG);
            utils::float_to_uint8(angles.second, bytes);
            Wire.write(bytes[0]);
            Wire.write(bytes[1]);
            break;
        default:
            logger("\nRequest on invalid command", LOG_WARN);
            return;
    }
}


void setup() {
    start_logger();

    for (int i = 0; i < 3; i++) {
        pinMode(led_pins[i], OUTPUT);
    }
    led_helper(0, 0, 0);

    // Initialize I2C buses
    logger("\nStarting I2C buses", LOG_DEBUG);
    Wire.begin(SLAVE_ADDRESS);
    sWires.first.begin();
    sWires.second.begin();

    // Set up I2C callbacks
    logger("\nSetting I2C callbacks", LOG_DEBUG);
    Wire.onReceive(on_receive);
    Wire.onRequest(on_request);


    // TODO: Check if the sensors are actually connected, and then initialize them

    // Indicate that the device is on but I2C has not been validated
    led_helper(1, 1, 0);
    logger("\nFinished init", LOG_DEBUG);
}

void loop() {
    static float previous_time= 0;
    if (micros() - previous_time >= SAMPLE_PERIOD) {
        angles = get_angles_pair();
        //logger("\nLeft: " + String(angles.first) + " Right: " + String(angles.second), LOG_DEBUG);
        previous_time = micros();
    }
}
