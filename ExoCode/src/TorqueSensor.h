/**
 * @file TorqueSensor.h
 * 
 * @brief Declares the class that is used to interface with a torque sensor.
 * 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef TorqueSensor_h
#define TorqueSensor_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Board.h"
#include "Arduino.h"

/**
 * @brief Class used to interface with a torque sensor
 */
class TorqueSensor
{
	public:
		TorqueSensor(unsigned int pin);
        
        /**
         * @brief Does the calibration, while do_calibration is true, and returns 0 when the calibration has finished.
         *  This gets the average of the readings during quite standing.
         * 
         * @param should the calibration be done
         *
         * @return 0 if the calibration is finished.
         */
        bool calibrate(bool do_calibration); 
		
        /**
         * @brief reads the pin and returns the calibrated value.
         * 
         * @return the calibrated torque reading
         */
        float read();
	    //MOVE BACK TO PRIVATE WHEN USING APP
        int _raw_reading; /**< Raw pin reading */
		
	private:
		int _pin; /**< Pin to read for the sensor */
        bool _is_used; /**< Flag indicating if the sensor is used */
        
        float _calibration;   /**< Stores the value used for calibration. This is a zero torque offset*/
        //int _raw_reading; /**< Raw pin reading */
		float _calibrated_reading; /**< Torque value with offset applied */
        
        const uint16_t _cal_time = 1000; /**< The time to do the initial calibration in ms*/  
        uint16_t _start_time; /**< time the calibration starts. */   
        bool _last_do_calibrate; /**< this when the calibration ends. */ //need to remember to reset this when cal finishes 
        float _zero_sum; /**< sum of values over the calibration period used for averaging. */  
        uint32_t _num_calibration_samples; /**< number of samples collected during calibration, denominator for averaging. */   
        
};
#endif
#endif