/** @file task_read_IMU.cpp
 * This is the implementation file for a class that has constructors and methods 
 * pertaining to the MPU6050 used in the gimbal project.
 * 
 * @author Jathun Somasundaram
 * @date 2023-Nov-27 
 * 
*/

#include "IMU.h"



/** @brief   Function that will initialize the MPU6050
 *  @details This function allows you to initialize by including the data line pin, clock line pin
 *           the IMU addr, and the power management register of the IMU. 
 *  @param   SDA_LINE Pin number on ESP32 with data line for I2C
 *  @param   SCL_LINE Pin number on ESP32 with clock line for I2C
 *  @param   IMU_ADDR Address of IMU peripheral
 *  @param   PWR_MGMT_1 Address of the power management register used to wake up the IMU 
*/
void IMU :: IMU_init (uint16_t SDA_LINE, uint16_t SCL_LINE, uint16_t IMU_ADDR, uint16_t PWR_MGMT_1 )
{
   Wire.begin(SDA_LINE, SCL_LINE, 400000); // sda, scl, clock speed of IMU
   Wire.beginTransmission(IMU_ADDR);
   Wire.write(PWR_MGMT_1);  // PWR_MGMT_1 register
   Wire.write(0);     // set to zero (wakes up the MPU−6050)
   Wire.endTransmission(true); //Releases i2c bus after tranmission ends for startup
}

/** @brief  Function that will return an offset for the pitch axis reading from
 *          the accelerometer
 *  @details This function takes the average of a couple hundred readings while the IMU is still.
 *           It returns the average which can be used as an offset value to calibrate raw readings later.
 *  @param MPU_ADDR Address of IMU peripheral
 * 
*/
int16_t IMU :: cal_acc_pitch (int16_t MPU_ADDR) // Pitch is now x-axis
{
uint16_t ACCEL_XOUT_H = 0x3B; ///< Register that contains the high-byte of the accelerometer's x-axis output
uint16_t count = 0;           ///< Keeps track of how many values have been summed during calibration
int16_t sum = 0;              ///< Keeps track of sum of all values during calibration


    for (int i = 0; i < 200; i++)
    {
        Wire.beginTransmission(MPU_ADDR); 
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6); // By requesting 6 registers, we get both high-byte and low-byte of x-axis, y-axis, and z-axis.

        if(Wire.available() == 0)
        {
            //Serial << "No Data Available" << endl;
        }
        else
        {
            //Serial << "Data Available from x-axis Accelerometer: " << Wire.available() << " Bytes" << endl;
        }

        AcX_raw = Wire.read() << 8 | Wire.read() / 16384; ///< AcX_raw is the x-axis accelerometer reading adjusted for sensitivity, but without an offset applied
        AcY_raw = Wire.read() << 8 | Wire.read() / 16384; ///< AcY_raw is the y-axis accelerometer reading adjusted for sensitivity, but without an offset applied
        AcZ_raw = Wire.read() << 8 | Wire.read() / 16384; ///< AcZ_raw is the z-axis accelerometer reading adjusted for sensitivity, but without an offset applied


        Wire.endTransmission(true);
        int16_t pitch_acc_raw = (atan(-1 * AcX_raw / sqrt(pow(AcY_raw, 2) + pow(AcZ_raw, 2))) * 180 / PI); ///< pitch_acc_raw is the pitch angle value from the accelerometer without the offset
        sum = sum + pitch_acc_raw;
        count ++;
        
    }
    
     pitch_offset_acc = sum/count; ///< Offset for pitch angle from accelerometer
     Serial << "Acc Pitch Offset is: " << pitch_offset_acc << endl;
    return pitch_offset_acc;

}


/** @brief  Function that will return the corrected pitch axis position from the accelerometer
 *  @details This function reads the necessary values from the register, computes the pitch angle, and applies
 *           the calibration offset found in the previous function. It then places the pitch angle value in a share
 *           that is used in the main file.
 *  @param MPU_ADDR Address of IMU peripheral          
*/

int16_t IMU :: read_acc_pitch (int16_t MPU_ADDR) // Pitch is now x-axis
{

uint16_t ACCEL_XOUT_H = 0x3B;

Wire.beginTransmission(MPU_ADDR);
Wire.write(ACCEL_XOUT_H);
Wire.endTransmission(false);
Wire.requestFrom(MPU_ADDR, 6);

if(Wire.available() == 0)
{
    //Serial << "No Data Available" << endl;

}
else
{
    //Serial << "Data Available from x-axis Accelerometer" << endl;
}

AcX_raw = Wire.read() << 8 | Wire.read() / 16384; 
AcY_raw = Wire.read() << 8 | Wire.read() / 16384;   
AcZ_raw = Wire.read() << 8 | Wire.read() / 16384;

Wire.endTransmission(true);

//Serial << AcX << endl;

int16_t pitch_acc = (atan(-1 * AcX_raw / sqrt(pow(AcY_raw, 2) + pow(AcZ_raw, 2))) * 180 / PI) - pitch_offset_acc; ///< pitch_acc is the pitch angle reading from the accelerometer with the offset applied

//Serial << "Pitch angle from Accelerometer (after correction): " << pitch_acc << endl;

pitch.put(pitch_acc);
return pitch_acc;



}

// ---------------------------------------------------------------------------------------
/** @brief  Function that will return an offset for the roll axis reading from
 *          the accelerometer
 *  @details This function takes the average of a couple hundred readings while the IMU is still.
 *           It returns the average which can be used as an offset value to calibrate raw readings later.
 *  @param MPU_ADDR Address of IMU peripheral
 * 
*/
int16_t IMU :: cal_acc_roll (int16_t MPU_ADDR) // Roll is now y-axis
{
uint16_t ACCEL_XOUT_H = 0x3B;
uint16_t count = 0;
int16_t sum = 0;

    for (int i = 0; i < 500; i++)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6);

        if(Wire.available() == 0)
        {
            //Serial << "No Data Available" << endl;
        }
        else
        {
            //Serial << "Data Available from y-axis Accelerometer: " << Wire.available() << " Bytes" << endl;
        }

        AcX_raw = Wire.read() << 8 | Wire.read() / 16384;
        AcY_raw = Wire.read() << 8 | Wire.read() / 16384;
        AcZ_raw = Wire.read() << 8 | Wire.read() / 16384;


        Wire.endTransmission(true);
        
        //int16_t roll_acc_raw = atan(AcY_raw/AcZ_raw) * 180/PI;
        int16_t roll_acc_raw = (atan(AcY_raw/sqrt(pow(AcX_raw,2) + pow(AcZ_raw,2))) * 180/PI); ///< roll_acc_raw is the roll angle value from the accelerometer without the offset
        sum = sum + roll_acc_raw;
        count ++;
        
    }
    
    roll_offset_acc = (sum/count); ///< Roll_offset_acc is the offset for the roll angle from the accelerometer
    return roll_offset_acc;

}

// ---------------------------------------------------------------------------------------

/** @brief  Function that will return the corrected roll axis position from the accelerometer
 *  @details This function reads the necessary values from the register, computes the roll angle, and applies
 *           the calibration offset found in the previous function. It then places the roll angle value in a share
 *           that is used in the main file.
 *  @param MPU_ADDR Address of IMU peripheral          
*/
int16_t IMU :: read_acc_roll (int16_t MPU_ADDR) // Roll is now y-axis
{
uint16_t ACCEL_XOUT_H = 0x3B;

Wire.beginTransmission(MPU_ADDR);
Wire.write(ACCEL_XOUT_H);
Wire.endTransmission(false);
Wire.requestFrom(MPU_ADDR, 6);

if(Wire.available() == 0)
{
    //Serial << "No Data Available" << endl;
}
else
{
    //Serial << "Data Available from y-axis Accelerometer" << endl;
}

AcX_raw = Wire.read() << 8 | Wire.read() / 16384;
AcY_raw = Wire.read() << 8 | Wire.read() / 16384;   
AcZ_raw = Wire.read() << 8 | Wire.read() / 16384;

Wire.endTransmission(true);

int16_t roll_acc = (atan(AcY_raw/sqrt(pow(AcX_raw,2) + pow(AcZ_raw,2))) * 180/PI) - roll_offset_acc; ///< roll_acc is the roll angle reading from the accelerometer with the offset applied

//Serial << "Roll angle from Accelerometer (after correction): " << roll_acc << endl;
return roll_acc;    

}

// ---------------------------------------------------------------------------------------

int16_t IMU :: cal_gyro_roll (int16_t MPU_ADDR) // Roll is now x-axis
{
uint16_t GYRO_XOUT_H = 0x43;
uint16_t count = 0;
uint16_t sum = 0;


    for (int i = 0; i < 200; i++)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 2);

        if(Wire.available() == 0)
        {
            //Serial << "No Data Available during calibration" << endl;
        }
        else
        {
            //Serial << "Data Available during calibration from pitch gyroscope: " << Wire.available() << " Bytes" << endl;
        }

        GyX_raw = Wire.read() << 8 | Wire.read() / 131;
        Wire.endTransmission(true);

        sum = sum + GyX_raw;
        count ++;
        
    }
    
    GyX_offset = sum/count;
    Serial << "Gyro Roll Offset is: " << GyX_offset << endl;
    return GyX_offset;

}

// ---------------------------------------------------------------------------------------


int16_t IMU :: read_gyro_roll (int16_t MPU_ADDR) // Roll is now x-axis
{
uint16_t GYRO_XOUT_H = 0x43;
uint16_t count = 0;
uint16_t sum = 0;

int16_t roll_gx;
int16_t roll_gx_prev = 0;

unsigned long current_time = 0;
unsigned long previous_time = 0;
unsigned long duration = 0;

roll_gx_prev = roll_gx;

previous_time = current_time;
current_time = millis();
duration = (current_time - previous_time)/1000;

Wire.beginTransmission(MPU_ADDR);
Wire.write(GYRO_XOUT_H);
Wire.endTransmission(false);
Wire.requestFrom(MPU_ADDR, 2);

    if(Wire.available() == 0)
        {
            //Serial << "No Data Available" << endl;
        }
    else
        {
            //Serial << "Data Available from x-axis gyroscope: " << Wire.available() << " Bytes" << endl;
        }

GyX_raw = Wire.read() << 8 | Wire.read() / 131;
Wire.endTransmission(true);

//Serial << "Duration of individual recording during readings: " << duration << endl;

GyX_raw = GyX_raw - GyX_offset;



roll_gx = roll_gx + GyX_raw*duration;
Serial << "Roll angle from Gyroscope (after correction): " << roll_gx << endl;
return roll_gx;
}

// --------------------------------------------------------------------------------------















int16_t IMU :: cal_gyro_pitch (int16_t MPU_ADDR)
{
uint16_t GYRO_YOUT_H = 0x45;
uint16_t count = 0;
uint16_t sum = 0;


    for (int i = 0; i < 200; i++)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_YOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 2);

        if(Wire.available() == 0)
        {
            //Serial << "No Data Available" << endl;
        }
        else
        {
            //Serial << "Data Available from x-axis Accelerometer: " << Wire.available() << " Bytes" << endl;
        }

        GyY_raw = Wire.read() << 8 | Wire.read() / 131;
        Wire.endTransmission(true);
        
        sum = sum + GyY_raw;
        count ++;
        
    }
    
    //GyY_offset = sum/count;
    return roll_gy_offset;

}

int16_t IMU :: cal_gyro_yaw (int16_t MPU_ADDR)
{
uint16_t GYRO_ZOUT_H = 0x47;
uint16_t count = 0;
uint16_t sum = 0;


    for (int i = 0; i < 200; i++)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(GYRO_ZOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 2);

        if(Wire.available() == 0)
        {
            //Serial << "No Data Available" << endl;
        }
        else
        {
            //Serial << "Data Available from x-axis Accelerometer: " << Wire.available() << " Bytes" << endl;
        }

        GyZ_raw = Wire.read() << 8 | Wire.read() / 131;
        Wire.endTransmission(true);
        
        sum = sum + GyZ_raw;
        count ++;
        
    }
    
    //GyZ_offset = sum/count;
    return yaw_gy_offset;

}

