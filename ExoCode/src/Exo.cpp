/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Exo.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the Exo
 * Takes the exo_data
 * Uses initializer list for legs.
 * Only stores these objects, and exo_data pointer.
 */
Exo::Exo(ExoData* exo_data)
: left_leg(true, exo_data)
, right_leg(false, exo_data) // constructor: uses initializer list for the legs.
{
    this->data = exo_data;
    
};

#endif