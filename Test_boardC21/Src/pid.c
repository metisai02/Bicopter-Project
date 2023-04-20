/*
 * pid.c
 *
 *  Created on: Apr 3, 2023
 *      Author: NHHanh
 */

#include "pid.h"

// PID variables declaration
float error;
float Pterm_pitch, Iterm_pitch, Dterm_pitch, last_error_pitch, PID_pitch_out;
float Pterm_roll, Iterm_roll, Dterm_roll, last_error_roll, PID_roll_out;
float Pterm_yaw, Iterm_yaw, Dterm_yaw, last_error_yaw, PID_yaw_out;

extern float abs_yaw_angle;



//-----------------------------------------------------------------------------
// calculate PID
//-----------------------------------------------------------------------------
void calculate_PID(float setpoint_roll, float setpoint_pitch, float setpoint_yaw, float roll_angle, float pitch_angle, float yaw_angle, PID_t *PID_out)
{
    // pitch PID calculations
    error = setpoint_pitch - pitch_angle * 12.5 - 1500;
    Pterm_pitch = error * Kp_pitch;
    Iterm_pitch += error;
    Dterm_pitch = (error - last_error_pitch) * Kd_pitch;
    last_error_pitch = error;
    PID_pitch_out = Pterm_pitch + (Iterm_pitch * Ki_pitch) + Dterm_pitch;
    if (PID_pitch_out > MAX_pitch_output)
        PID_pitch_out = MAX_pitch_output;
    if (PID_pitch_out < -MAX_pitch_output)
        PID_pitch_out = -MAX_pitch_output;

    // roll PID calculations
    error = setpoint_roll + roll_angle * 12.5 - 1500;
    Pterm_roll = error * Kp_roll;
    Iterm_roll += error;
    Dterm_roll = (error - last_error_roll) * Kd_roll;
    last_error_roll = error;
    PID_roll_out = Pterm_roll + (Iterm_roll * Ki_roll) + Dterm_roll;
    if (PID_roll_out > MAX_roll_output)
        PID_roll_out = MAX_roll_output;
    if (PID_roll_out < -MAX_roll_output)
        PID_roll_out = -MAX_roll_output;

    // yaw PID calculations
    error = (setpoint_yaw + abs_yaw_angle) * 12.5;
    Pterm_yaw = error * Kp_yaw;
    Iterm_yaw += error;
    Dterm_yaw = (error - last_error_yaw) * Kd_yaw;
    last_error_yaw = error;
    PID_yaw_out = Pterm_yaw + (Iterm_yaw * Ki_yaw) + Dterm_yaw;
    if (PID_yaw_out > MAX_yaw_output)
        PID_yaw_out = MAX_yaw_output;
    if (PID_yaw_out < -MAX_yaw_output)
        PID_yaw_out = -MAX_yaw_output;

    PID_out->PID_roll_out = PID_roll_out;
    PID_out->PID_pitch_out = PID_pitch_out;
    PID_out->PID_yaw_out = PID_yaw_out;
}
