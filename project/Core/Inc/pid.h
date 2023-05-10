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

#define dt 0.01

void pid_calculate(float Error, float P, float I, float D, float PrevError, float PrevIterm);
void pid_roll(uint16_t roll_rc, float roll_angle, float roll_rate, PID_t *PID_out);
void pid_pitch(uint16_t pitch_rc, float pitch_angle, float pitch_rate, PID_t *PID_out);
void pid_yaw(uint16_t yaw_rc, float yaw_rate, PID_t *PID_out);

#endif /* PID_H_ */
