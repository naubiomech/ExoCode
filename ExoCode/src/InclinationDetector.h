#ifndef INCLINATION_DETECTOR_H
#define INCLINATION_DETECTOR_H
#include <Arduino.h>
#include <utility>

enum class Inclination : uint8_t
{
    Decline = 0,
    Level = 1,
    Incline = 2,
};

/**
 * @brief Given stance and ankle angle data, return the current estimate of inclination
 * 
 */
class InclinationDetector
{
    public:
    InclinationDetector();

    /**
     * @brief Set the decline angle object
     * 
     * @param threshold 
     */
    void set_decline_angle(float threshold);
    /**
     * @brief Set the incline angle threshold
     * 
     * @param threshold 
     */
    void set_incline_angle(float threshold);

    /**
     * @brief check the current data
     * 
     * @param is_stance 
     * @param norm_angle 
     * @return Inclination 
     */
    Inclination check(const bool is_stance, const float norm_angle);
    
    private:
    /* Exposed */

    std::pair<float, float> _decline_stance_phase_percent;
    float _incline_angle_theshold;
    float _decline_angle_threshold;

    /* Internal */

    bool _returned_this_stance;
    Inclination _prev_estimate;
    bool _prev_stance;
    float _prev_stance_time;
    float _current_stance_start;

    enum class Edge: int
    {
        None = 0,
        Rising = 1,
        Falling = 2, 
    };
    /**
     * @brief Check the current stance for an edge, and handle edge tracking state
     * 
     * @param is_stance 
     * @return Edge 
     */
    Edge _check_for_edge(const bool is_stance);
    /**
     * @brief Update the stance phase estimation and return a current estimate.
     * 
     * @param edge
     * @param is_stance 
     * @return float Zero if in swing, stance phase percent if in stance (0-100)
     */
    float _update_stance_phase(const Edge edge, const bool is_stance);
    /**
     * @brief Assuming that there is a rising edge, check if the angle is lower 
     * than the threshold
     * 
     * @param norm_angle 
     * @return true Incline
     * @return false Not Incline
     */
    bool _incline_check(const float norm_angle);
    /**
     * @brief Assuming that the stance phase percentage is correct, check if the range 
     * lower than the threshold
     * 
     * @param norm_angle 
     * @return true Decline
     * @return false Not Decline
     */
    bool _decline_check(const float norm_angle);
};

#endif