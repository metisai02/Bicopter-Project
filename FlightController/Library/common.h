/*
 * common.h
 *
 *  Created on: Apr 14, 2023
 *      Author: DELL
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "HMC5883L.h"
//#include "mpu6050.h"
#include "mpu6051_ver1.h"
#include "IMU.h"
#include "pid.h"

#define RC_deadband 4
//rate in microsecond
#define FLOW_RATE 13
#define HCSR_RATE 60

//#define M_PI  3.1415926535f

#define Motor_MIN 1150
#define Motor_MAX 1800

#define DMA_RX_BUFFER_SIZE 9

#define DEG_TO_RAD M_PI/180.0f
#define RAD_TO_DEG 180.0f/M_PI

#endif /* COMMON_H_ */
