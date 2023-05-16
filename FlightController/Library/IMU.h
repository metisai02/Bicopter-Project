#ifndef __IMU_H
#define __IMU_H
/*
imu_quaternion
imu_brokking
*/
//#define imu_quaternion

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "mpu6050.h"
#include "HMC5883L.h"



float safe_asin(float v);
extern float IMU_values[9];
//extern int16_t IMU_unscaled[6];
extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;	 //��λ ��

extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz;	
extern volatile float acc_vector;  
void FreeIMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az);
//Mini IMU AHRS 
void IMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz);
void IMU_getValues(float *values);
void IMU_getValues_2(float *values);
void imu_hardware_setup(void);
//void Initialize_Q(void);
//void IMU_getRollPitchYaw(float *angles);
float LPF(float sample,float pre_value, float cut_off,float dt);
void IMU_getAttitude(float *RPY,float *rate_RPY);
void Initialize_Q(void);
//void Initialize_Q(void);
void IMU_getQ(float *q,volatile float IMU_values[9]);
//get roll pitch yaw from quaternion
void IMU_getRollPitchYaw(float *angles, float *qa);
float average_filter(float buffer[], uint8_t buffer_size, float sample, uint8_t *count);
float invSqrt(float x);
#endif

