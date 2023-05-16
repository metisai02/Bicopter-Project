/*
 * mpou6050.c
 *
 *  Created on: Apr 3, 2023
 *      Author: NHHanh
 */



#include "mpu6051_ver1.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define CONFIG_REG  0x1A
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;

uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        Data = 0x06;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, i2c_timeout);


        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x01;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x18;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}


void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, SixAxis *DataStruct) {
    uint8_t Rec_Data[14];

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Ax = ((int16_t) (Rec_Data[0] << 8 | Rec_Data[1])) /1671.83f;
    DataStruct->Ay = ((int16_t) (Rec_Data[2] << 8 | Rec_Data[3])) /1671.83f;
    DataStruct->Az = ((int16_t) (Rec_Data[4] << 8 | Rec_Data[5])) /1671.83f;

    DataStruct->Gx = ((int16_t) (Rec_Data[8] << 8 | Rec_Data[9])) / 16.4f;
    DataStruct->Gy = ((int16_t) (Rec_Data[10] << 8 | Rec_Data[11])) / 16.4f;
    DataStruct->Gz = ((int16_t) (Rec_Data[12] << 8 | Rec_Data[13])) / 16.4f;

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
     DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
     DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

     DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
     DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
     DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

     DataStruct->Ax = DataStruct->Accel_X_RAW /1671.83f;
     DataStruct->Ay = DataStruct->Accel_Y_RAW /1671.83f;
     DataStruct->Az = DataStruct->Accel_Z_RAW / 1671.83f;

     DataStruct->Gx = DataStruct->Gyro_X_RAW / 16.4f;
     DataStruct->Gy = DataStruct->Gyro_Y_RAW / 16.4f;
     DataStruct->Gz = DataStruct->Gyro_Z_RAW / 16.4f;

#ifdef IMU_SOFTWARE_FIXED
    DataStruct->Gx += G_X_OFFSET;
    DataStruct->Gy += G_Y_OFFSET;
    DataStruct->Gz += G_Z_OFFSET;

    DataStruct->Ax += A_X_OFFSET;
    DataStruct->Ay += A_Y_OFFSET;
    DataStruct->Az += A_Z_OFFSET;
#endif
    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
        DataStruct->AngleZ =roll;
    } else {
        roll = 0.0;
        DataStruct->AngleZ =roll;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);

}
float g_Yaw, g_Pitch, g_Roll;
SixAxis avm_euler;
void IMU_Comput(SixAxis cache) {
	static float g_q0 = 1, g_q1 = 0, g_q2 = 0, g_q3 = 0;   //Quaternion
	static float g_exInt = 0, g_eyInt = 0, g_ezInt = 0;


    float norm;     //模
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrt(cache.KalmanAngleX*cache.KalmanAngleX + cache.KalmanAngleY*cache.KalmanAngleY + cache.AngleZ*cache.AngleZ);     //取模


    cache.KalmanAngleX = cache.KalmanAngleX / norm;
    cache.KalmanAngleY = cache.KalmanAngleY / norm;
    cache.AngleZ = cache.AngleZ / norm;


    vx = 2 * (g_q1 * g_q3 - g_q0 * g_q2);
    vy = 2 * (g_q0 * g_q1 + g_q2 * g_q3);
    vz = g_q0*g_q0 - g_q1*g_q1 - g_q2*g_q2 + g_q3*g_q3;


    ex = (cache.KalmanAngleY * vz - cache.AngleZ * vy);
    ey = (cache.AngleZ * vx - cache.KalmanAngleX * vz);
    ez = (cache.KalmanAngleX * vy - cache.KalmanAngleY * vx);


    g_exInt += ex * Ki;
    g_eyInt += ey * Ki;
    g_ezInt += ez * Ki;


    cache.Gx += Kp * ex + g_exInt;
    cache.Gy += Kp * ey + g_eyInt;
    cache.Gz += Kp * ez + g_ezInt;


    g_q0 += (-g_q1 * cache.Gx - g_q2 * cache.Gy - g_q3 * cache.Gz) * halfT;
    g_q1 += (g_q0 * cache.Gx + g_q2 * cache.Gz - g_q3 * cache.Gy) * halfT;
    g_q2 += (g_q0 * cache.Gy - g_q1 * cache.Gz + g_q3 * cache.Gx) * halfT;
    g_q3 += (g_q0 * cache.Gz + g_q1 * cache.Gy - g_q2 * cache.Gx) * halfT;

    //正常化四元
    norm = sqrt(g_q0*g_q0 + g_q1*g_q1 + g_q2*g_q2 + g_q3*g_q3);
    g_q0 = g_q0 / norm;
    g_q1 = g_q1 / norm;
    g_q2 = g_q2 / norm;
    g_q3 = g_q3 / norm;

    g_Pitch = asin(-2 * g_q1 * g_q3 + 2 * g_q0 * g_q2) * 57.3;
    g_Roll = atan2(2 * g_q2 * g_q3 + 2 * g_q0 * g_q1, -2 * g_q1*g_q1 - 2 * g_q2*g_q2 + 1) * 57.3;
    g_Yaw = atan2(2 * (g_q1 * g_q2 + g_q0 * g_q3), g_q0*g_q0 + g_q1*g_q1 - g_q2*g_q2 - g_q3*g_q3) * 57.3;
}
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
