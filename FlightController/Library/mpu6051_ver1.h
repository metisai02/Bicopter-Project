/*
 * mpu6050.h
 *
 *  Created on: Apr 3, 2023
 *      Author: NHHanh
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "math.h"
// MPU6050 structure
#define IMU_SOFTWARE_FIXED

#define G_X_OFFSET 1.5853f
#define G_Y_OFFSET 0.7926f
#define G_Z_OFFSET 2.0121f

#define A_X_OFFSET 0
#define A_Y_OFFSET 0
#define A_Z_OFFSET 0.2f

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;
    double AngleZ;
    double KalmanAngleX;
    double KalmanAngleY;

} SixAxis,*pSixAxis;
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

#define Kp      100.0f
#define Ki      0.002f
#define halfT   0.001f

extern float g_Yaw, g_Pitch, g_Roll;
extern SixAxis avm_euler;



uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, SixAxis *DataStruct);
void IMU_Comput(SixAxis cache);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);


#endif /* INC_MPU6050_H_ */
