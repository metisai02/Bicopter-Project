/*
 * pid.h
 *
 *  Created on: Apr 20, 2023
 *      Author: DELL
 */

#ifndef PID_H_
#define PID_H_

#include "stm32f1xx_hal.h"

typedef struct
{
    float PID_roll_out;
    float PID_pitch_out;
    float PID_yaw_out;
} PID_t;

//+40deg =  2070 us,, -40deg = 1160 us,, 910 us range,, 1615 us = 0deg
#define MAX_pitch_output 454 // 454
#define MAX_roll_output 454
#define MAX_yaw_output 454

// PID constant values for all three axis
//#define Kp_pitch 0.25 //.5
//#define Ki_pitch 0.0
//#define Kd_pitch 3.3
//
//#define Kp_roll 0.4 //.2
//#define Ki_roll 0.0
//#define Kd_roll 4.8
//
//#define Kp_yaw 0.15 //.5
//#define Ki_yaw 0.0
//#define Kd_yaw 3.0

#define dt 0.01

void calculate_PID(uint16_t roll_rc, uint16_t pitch_rc, uint16_t yaw_rc, float roll_angle, float pitch_angle, float yaw_angle, PID_t *PID_out);

#endif /* PID_H_ */
