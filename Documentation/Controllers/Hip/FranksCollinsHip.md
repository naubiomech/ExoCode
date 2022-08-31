# Franks Collins Hip Controller

## Description
Provides Hip assistance based on a series of splines that have an extension and flexion portion.
Uses percent gait, requiring heel FSR.
Based on: 

Franks, P. W., Bryan, G. M., Martin, R. M., Reyes, R., Lakmazaheri, A. C., & Collins, S. H. (2021). 
[Comparing optimized exoskeleton assistance of the hip, knee, and ankle in single and multi-joint configurations](https://www.cambridge.org/core/journals/wearable-technologies/article/comparing-optimized-exoskeleton-assistance-of-the-hip-knee-and-ankle-in-single-and-multijoint-configurations/9FBC1580F11614B388BE621D716800AD). 
Wearable Technologies, 2.

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- mass - User mass in kg used for denormalizing torque.
- trough_normalized_torque_Nm_kg - Largest extension torque normalized to user mass (Nm/kg).
- peak_normalized_torque_Nm_kg - largest flexion torquenormalized to user mass (Nm/kg).
- start_percent_gait - percent gait to start the torque pattern so it doesn't have a discontinuity at heel strike.
- trough_onset_percent_gait - percent gait to start applying extension torque.
- trough_percent_gait - percent gait to apply the largest extension torque.
- mid_time - transition point between extension and flexion to apply 0 torque.
- mid_duration - the duration of the transition pause as a percent of the gait cycle.
- peak_percent_gait - percent gait to apply the largest flexion torque.
- peak_offset_percent_gait - percent gait to stop applying flexion torque.
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls