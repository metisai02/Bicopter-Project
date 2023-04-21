

#include "main.h"
#include "pid.h"
#include "motor.h"

void calculate_motor_output(uint16_t *esc_right, uint16_t *esc_left, uint16_t *servo_right, uint16_t *servo_left, uint16_t throttle_rc, PID_t *pid)
{
    // value PWM
    *esc_right = throttle_rc + pid->PID_roll_out;
    *esc_left = throttle_rc - pid->PID_roll_out;
    *servo_right = 1500 + pid->PID_pitch_out - pid->PID_yaw_out + SERVO_RIGHT_OFFSET;
    *servo_left = 1500 - pid->PID_pitch_out - pid->PID_yaw_out + SERVO_LEFT_OFFSET;
}
