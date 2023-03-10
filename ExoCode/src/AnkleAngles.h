#ifndef ANKLE_ANGLE_H
#define ANKLE_ANGLE_H

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include <Arduino.h>

//TODO: Add angle calibration

/**
 * @brief Class to interface with a microcontroller that samples angle data. Singleton. 
 * 
 */
class AnkleAngles
{
    protected:
    AnkleAngles() {}
    static AnkleAngles* _instance;

    public:
    AnkleAngles(AnkleAngles& other) = delete;
    void operator=(const AnkleAngles&) = delete;
    static AnkleAngles* GetInstance();
    /**
     * @brief Initialize the ankle sensor, and perform a handshake.
     * 
     * @param timeout Timeout before giving up on handshake in ms
     * @return true 
     * @return false 
     */
    bool init();
    /**
     * @brief Get the left or right angle. Returns a ratiometric value [0, 1]
     * 
     * @return float angle
     */
    float get(bool left);

    private:
    /**
     * @brief Check if the ankle sensor is initialized.
     * 
     */
    bool _is_initialized = false;
    /**
     * @brief Right analog pin
     * 
     */
    const int _right_pin = A17;
    /**
     * @brief Left analog pin
     * 
     */
    const int _left_pin = A16;
};


#endif
#endif