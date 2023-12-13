/** @file motor_obj.h
 * This is the header file for a class that has constructors and methods 
 * pertaining to motors used in the gimbal project.
 * 
 * @author Jathun Somasundaram
 * @date 2023-Nov-23 
 * 
*/

#ifndef _motor_obj_
#define _motor_obj_

#include <Arduino.h>
#include <PrintStream.h>

/** @brief Class used for motor control for a gimbal
 * 
*/

class Motor
{
    protected:
    float motor_time_tau = 6.08; // milliseconds


    public:
        void init(uint8_t in1_pin, uint8_t in2_pin, uint32_t freq, uint8_t res);
        void spin (uint8_t ch1_dc, uint8_t ch2_dc);
        void brake (void);
    
};




#endif