#if defined(ARDUINO_ARDUINO_NANO33BLE)

#include "InclineDetector.h"
#include "Utilities.h"

#define INCLINE_DETECTOR_DEBUG 0

InclineDetector::InclineDetector(float alpha, float threshold)
{
    #if INCLINE_DETECTOR_DEBUG
    Serial.println("InclineDetector::InclineDetector()");
    Serial.print("alpha: ");
    Serial.println(alpha);
    Serial.print("threshold: ");
    Serial.println(threshold);
    #endif

    _alpha = alpha;
    _threshold = threshold;
    _avgPressure = -1;
    _state = Level;
    _smooth_demeanedPressure = 0;
}

incline_state_t InclineDetector::run(float pressure)
{
    if (_avgPressure == -1)
    {
        _avgPressure = pressure;
    }
    else
    {
        _avgPressure = utils::ewma(pressure, _avgPressure, _alpha);
    }
    float demeanedPressure = pressure - _avgPressure;

    _smooth_demeanedPressure = utils::ewma(demeanedPressure, _smooth_demeanedPressure, _alpha);

    if (_smooth_demeanedPressure > _threshold)
    {
        _state = Decline;
    }
    else if (_smooth_demeanedPressure < -_threshold)
    {
        _state = Incline;
    }
    else
    {
        _state = Level;
    }

    #if INCLINE_DETECTOR_DEBUG
    Serial.print("Pressure: ");
    Serial.print(_smooth_demeanedPressure);
    Serial.print("\t");
    Serial.print("High: ");
    Serial.print(_threshold);
    Serial.print("\t");
    Serial.print("Low: ");
    Serial.println(-_threshold);

    #endif

    return _state;
}

incline_state_t InclineDetector::getInclineState()
{
    return _state;
}

void InclineDetector::setAlpha(float alpha)
{
    _alpha = alpha;
}

void InclineDetector::setThreshold(float threshold)
{
    _threshold = threshold;
}

#endif