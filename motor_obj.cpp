/** @file motor_obj.h
 * This is the header file for a class that has constructors and methods 
 * pertaining to motors used in the gimbal project.
 * 
 * @author Jathun Somasundaram
 * @date 2023-Nov-23 
 * 
*/

#include "motor_obj.h"

/** @brief  Method to initialize a motor object with specific parameters
 * 
 *  @details This method will allow a motor object to be initialized 
 *           with the inputs listed below, which will allow for 3 different motors
 * 
 *  @param   in1_pin This is the pin number for channel 1 of the motor driver
 *  @param   in2_pin This is the pin number for channel 2 of the motor driver
 *  @param   freq    This is the pwm frequency for both channels
 *  @param   res     This is the resolution of the pwm
*/



void Motor :: init(uint8_t in1_pin, uint8_t in2_pin, uint32_t freq, uint8_t res)
{
    uint8_t in1_channel = 0;
    uint8_t in2_channel = 1;
    
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);

    ledcSetup(in1_channel, freq, res);
    ledcSetup(in2_channel, freq, res);

    ledcAttachPin(in1_pin, in1_channel);
    ledcAttachPin(in2_pin, in2_channel);
}

void Motor :: spin (uint8_t ch1_dc, uint8_t ch2_dc)
{
    uint8_t in1_channel = 0;
    uint8_t in2_channel = 1;
    
    if (ch1_dc > 0 && ch2_dc == 0)
    {
        Serial << "Motor spinning forward" << endl;
    }

    if (ch1_dc == 0 && ch2_dc > 0)
    {
        Serial << "Motor spinning backwards" << endl;
    }

   
    ledcWrite (in1_channel, ch1_dc);
    ledcWrite (in2_channel, ch2_dc);
}


void Motor :: brake (void)
{
    uint8_t in1_channel = 0;
    uint8_t in2_channel = 1;
    
    Serial << "Motor braked" << endl;
    
    ledcWrite (in1_channel, 255);
    ledcWrite (in2_channel, 255);
}



