/*
 * motor.c
 *
 *  Created on: Apr 21, 2023
 *      Author: DELL
 */

#include "main.h"
#include "pid.h"
#include "motor.h"
#if (TUNING)
extern float SERVO_RIGHT_OFFSET; // Servo offset for right servo
extern float SERVO_LEFT_OFFSET;
#endif
void calculate_motor_output(uint16_t *esc_right, uint16_t *esc_left, uint16_t *servo_right, uint16_t *servo_left, uint16_t throttle_rc, PID_t *pid)
{

    // value PWM
    *esc_right = throttle_rc + pid->PID_roll_out;
    *esc_left = throttle_rc - pid->PID_roll_out;
    *servo_right = 3750 + pid->PID_pitch_out - pid->PID_yaw_out + SERVO_RIGHT_OFFSET;
    *servo_left = 3750 - pid->PID_pitch_out - pid->PID_yaw_out + SERVO_LEFT_OFFSET;
    if (*esc_right < 2500)
    {
        *esc_right = 2500;
    }
    if (*esc_right > 5000)
    {
        *esc_right = 5000;
    }
    if (*esc_left < 2500)
    {
        *esc_left = 2500;
    }
    if (*esc_left > 5000)
    {
        *esc_left = 5000;
    }
}
