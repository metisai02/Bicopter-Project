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


#define MAX_pitch_output 400 // 454
#define MAX_roll_output 400
#define MAX_yaw_output 400

// PID constant values for all three axis
// #define Kp_pitch 0.25 //.5
// #define Ki_pitch 0.0
// #define Kd_pitch 3.3
//
// #define Kp_roll 0.4 //.2
// #define Ki_roll 0.0
// #define Kd_roll 4.8
//
// #define Kp_yaw 0.15 //.5
// #define Ki_yaw 0.0
// #define Kd_yaw 3.0

#define dt 0.01

void pid_calculate(float Error, float P, float I, float D, float PrevError, float PrevIterm);
void pid_roll(uint16_t roll_rc, float roll_angle, float roll_rate, PID_t *PID_out);
void pid_pitch(uint16_t pitch_rc, float pitch_angle, float pitch_rate, PID_t *PID_out);


#endif /* PID_H_ */
