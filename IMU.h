/** @file task_read_IMU.h
 * This is the header file for a class that has constructors and methods 
 * pertaining to the MPU6050 used in the gimbal project.
 * 
 * @author Jathun Somasundaram
 * @date 2023-Nov-27 
 * 
*/

#ifndef _IMU_
#define _IMU_

#include <Arduino.h>
#include <PrintStream.h>
#include <Wire.h>
#include <SPI.h>

#include "taskshare.h"
#include "taskqueue.h"

extern Queue <int16_t> roll_angle_acc;
extern Queue <int16_t> pitch_angle_acc;

extern Queue <int16_t> yaw_angle_gyro;
extern Queue <int16_t> roll_angle_gyro;
extern Queue <int16_t> pitch_angle_gyro;

extern Queue <int16_t> yaw;
extern Queue <int16_t> roll;
extern Share <int16_t> pitch;

class IMU
{
    protected:
        int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;   // Adjusted readings (Linear Acceleration )
        int16_t AcX_raw, AcY_raw, AcZ_raw, GyX_raw, GyY_raw, GyZ_raw; // Raw readings

        int16_t pitch_offset_acc, roll_offset_acc;

        int32_t GyX_offset, GyY_offset, GyZ_offset;
        int32_t pitch_gy_offset, roll_gy_offset, yaw_gy_offset;

    public:
        void IMU_init (uint16_t, uint16_t, uint16_t, uint16_t);
        
        int16_t cal_acc_roll (int16_t); // x-axis
        int16_t read_acc_roll (int16_t);

        int16_t cal_acc_pitch (int16_t); // y-axis
        int16_t read_acc_pitch (int16_t);       
       
        int16_t cal_gyro_roll(int16_t);
        int16_t read_gyro_roll(int16_t);

        int16_t cal_gyro_pitch(int16_t);
        int16_t read_gyro_pitch(int16_t);

        int16_t cal_gyro_yaw(int16_t); // z-axis
        int16_t read_gyro_yaw(int16_t);
        

       


        





};

#endif


