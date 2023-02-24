#include "AnkleAngles.h"
#include "Logger.h"

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

AnkleAngles* AnkleAngles::_instance = nullptr;

AnkleAngles* AnkleAngles::GetInstance()
{
    if (_instance == nullptr) {
        _instance = new AnkleAngles();
        _instance->init();
    }
    return _instance;
}

bool AnkleAngles::init()
{
    pinMode(_left_pin, INPUT);
    pinMode(_right_pin, INPUT);
    _is_initialized = true;
}

float AnkleAngles::get(bool left)
{
    if (!_is_initialized) {
        return 0;
    }
    const int adc_counts = left ? analogRead(_left_pin):analogRead(_right_pin);
    const float ratio = adc_counts / 4095.0f;
    // TODO: Check for errors
    return ratio;
}


#endif