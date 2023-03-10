#include "InclinationDetector.h"
#include <Arduino.h> // millis()
#include "Logger.h"
#include "LogLevels.h"

#define INCLINATION_DETECTOR_DEBUG 0

/* Default detection parameters */

static constexpr float k_decline_angle_threshold{0.6f};
static constexpr float k_incline_angle_theshold{0.3f};


InclinationDetector::InclinationDetector()
{
    _incline_angle_theshold = k_incline_angle_theshold;
    _decline_angle_threshold = k_decline_angle_threshold;

    _returned_this_stance = true;
    _prev_estimate = Inclination::Level;
    _prev_stance = true;
    _prev_stance_time = 1;
    _current_stance_start = millis();
}

void InclinationDetector::set_incline_angle(float threshold)
{
    _incline_angle_theshold = threshold;
}

void InclinationDetector::set_decline_angle(float threshold)
{
    _decline_angle_threshold = threshold;
}

InclinationDetector::Edge InclinationDetector::_check_for_edge(const bool is_stance)
{
    #if INCLINATION_DETECTOR_DEBUG
    //logger::println("Checking for edge with (is_stance): "+ String(is_stance), LogLevel::Debug);
    #endif

    Edge edge = Edge::None;
    if (is_stance > _prev_stance)
    {
        edge = Edge::Rising;
    } else if (is_stance < _prev_stance)
    {
        edge = Edge::Falling;
    }

    _prev_stance = is_stance;
    return edge;
}
float InclinationDetector::_update_stance_phase(const InclinationDetector::Edge edge, const bool is_stance)
{
    #if INCLINATION_DETECTOR_DEBUG
    logger::println("Updating stance phase with (edge, is_stance): " + String(static_cast<int>(edge)) + ", " + String(is_stance), LogLevel::Debug);
    #endif

    if (!is_stance && edge != Edge::Falling)
    {
        return 0;
    }


    const float now = static_cast<float>(micros())/1000;
    if (edge == Edge::Rising)
    {
        #if INCLINATION_DETECTOR_DEBUG
        logger::println("Updating _current_stance_start with (millis): " + String(now), LogLevel::Debug);
        #endif
        _current_stance_start = now;
        
        return 0;
    } else if (edge == Edge::Falling)
    {
        const float stance_time = now - _current_stance_start;
        // TODO: Check for bad stance phase (too short, or overflow error)
        #if INCLINATION_DETECTOR_DEBUG
        logger::println("Updating _prev_stance_time with (stance_time): " + String(stance_time), LogLevel::Debug);
        #endif
        _prev_stance_time = stance_time;
        return 100;
    } else if (edge == Edge::None)
    {   
        const float stance_phase = (now-_current_stance_start) / _prev_stance_time;
        #if INCLINATION_DETECTOR_DEBUG
        logger::println("Updating stance phase with (millis, current start, prev time, phase): " + String(now) + ", "
                + String(_current_stance_start) + ", " + String(_prev_stance_time) + ", " + String(stance_phase), LogLevel::Debug);
        #endif
        // TODO: Check for overflow
        return stance_phase;
    }
}
bool InclinationDetector::_incline_check(const float norm_angle)
{
    const bool is_incline = norm_angle < _incline_angle_theshold;

    #if INCLINATION_DETECTOR_DEBUG
    //logger::println("Incline check with (angle, is_incline): " + String(norm_angle) + ", " + String(is_incline), LogLevel::Debug);
    #endif

    return is_incline;
}
bool InclinationDetector::_decline_check(const float norm_angle)
{
    const bool is_decline = norm_angle < _decline_angle_threshold;

    #if INCLINATION_DETECTOR_DEBUG
    //logger::println("Decline check with (angle, is_decline): " + String(norm_angle) + ", " + String(is_decline), LogLevel::Debug);
    #endif

    return is_decline;
}

Inclination InclinationDetector::check(const bool is_stance, const float norm_angle)
{
    #if INCLINATION_DETECTOR_DEBUG
    logger::println("Checking for inclination with (is_stance, norm_angle): " + String(is_stance) + ", " + String(norm_angle), LogLevel::Debug);
    #endif

    const Edge edge = _check_for_edge(is_stance);
    //const float stance_phase_percent = _update_stance_phase(edge, is_stance);
    //_update_angles_at(stance_phase_percent, norm_angle, edge);

    if (edge == Edge::Rising) {
        #if INCLINATION_DETECTOR_DEBUG
        logger::println("Rising Edge with (is_stance, norm_angle): " + String(is_stance) + ", " + String(norm_angle), LogLevel::Debug);
        #endif
        _returned_this_stance = false;
        const bool is_incline = _incline_check(norm_angle);
        if (is_incline) {
            _returned_this_stance = true;
            #if INCLINATION_DETECTOR_DEBUG
            logger::println("Returning Incline", LogLevel::Debug);
            #endif
            _prev_estimate = Inclination::Incline;
            return Inclination::Incline;
        }
    }

    if (edge == Edge::Falling && !_returned_this_stance) {
        #if INCLINATION_DETECTOR_DEBUG
        logger::println("Falling Edge with (is_stance, norm_angle): " + String(is_stance) + ", " + String(norm_angle), LogLevel::Debug);
        #endif
        const bool is_decline = _decline_check(norm_angle);
        if (is_decline) {
            #if INCLINATION_DETECTOR_DEBUG
            logger::println("Returning Decline", LogLevel::Debug);
            #endif
            _prev_estimate = Inclination::Decline;
            return Inclination::Decline;
        } else {
            #if INCLINATION_DETECTOR_DEBUG
            logger::println("Returning Level", LogLevel::Debug);
            #endif
            _prev_estimate = Inclination::Level;
            return Inclination::Level;
        }
    }

    return _prev_estimate;

    // #if INCLINATION_DETECTOR_DEBUG
    // logger::println("Check got (edge, stance_phase_percent): " + String(static_cast<int>(edge)) + ", " + String(stance_phase_percent), LogLevel::Debug);
    // #endif

    // if (!_may_update(edge, is_stance))
    // {
    //     return _statefully_from_check(_prev_estimate, false);
    // }
    // if (edge == Edge::Falling)
    // {
    //     _returned_this_stance = false;
    // }

    // #if INCLINATION_DETECTOR_DEBUG
    // logger::println("==============================================================", LogLevel::Debug);
    // #endif

    // if ((edge == Edge::Rising) && _incline_check(norm_angle))
    // {
    //     return _statefully_from_check(Inclination::Incline, true);
    // } else if (edge == Edge::Falling && _decline_check(norm_angle))
    // {
    //     return _statefully_from_check(Inclination::Decline, true);
    // } else if (edge == Edge::Falling)
    // {
    //     return _statefully_from_check(Inclination::Level, true);
    // }
    // return _statefully_from_check(_prev_estimate, false);
}