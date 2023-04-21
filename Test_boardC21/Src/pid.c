/*
 * pid.c
 *
 *  Created on: Apr 20, 2023
 *      Author: DELL
 */

#include "pid.h"

#include "pid.h"

// PID variables declaration
float error;
float Pterm_pitch, Iterm_pitch, Dterm_pitch, last_error_pitch, PID_pitch_out;
float Pterm_roll, Iterm_roll, Dterm_roll, last_error_roll, PID_roll_out;
float Pterm_yaw, Iterm_yaw, Dterm_yaw, last_error_yaw, PID_yaw_out;

// setpoint declaration
float setpoint_roll = 0;
float setpoint_pitch = 0;
float setpoint_yaw = 0;

extern float abs_yaw_angle;

float Kp_pitch = 0.25; //.5
float Ki_pitch = 0.0;
float Kd_pitch = 3.3;

float Kp_roll = 0.4; //.2
float Ki_roll = 0.0;
float Kd_roll = 4.8;

float Kp_yaw = 0.15; //.5
float Ki_yaw = 0.0;
float Kd_yaw = 3.0;

//-----------------------------------------------------------------------------
// calculate PID
//-----------------------------------------------------------------------------
void calculate_PID(uint16_t roll_rc, uint16_t pitch_rc, uint16_t yaw_rc, float roll_angle, float pitch_angle, float yaw_angle, PID_t *PID_out)
{
    // set a dead band at zero to improve stability on roll
    if (roll_rc > 1500 || roll_rc < 1440)
    {
        setpoint_roll = roll_rc;
    }
    else
    {
        setpoint_roll = 1500;
    }

    // set a dead band at zero to improve stability on pitch
    if (pitch_rc > 1540 || pitch_rc < 1440)
    {
        setpoint_pitch = pitch_rc;
    }
    else
    {
        setpoint_pitch = 1500;
    }

    // set a dead band at zero to improve stability on yaw
    if (yaw_rc > 1390)
    {
        setpoint_yaw = setpoint_yaw + 0.7;
    }
    else if (yaw_rc < 1357)
    {
        setpoint_yaw = setpoint_yaw - 0.7;
    }

    // pitch PID calculations
    error = setpoint_pitch - pitch_angle * 12.5 - 1500;
    Pterm_pitch = error * Kp_pitch;
    Iterm_pitch += (error + last_error_pitch) * Ki_pitch * dt / 2;
    Dterm_pitch = (error - last_error_pitch) * Kd_pitch / dt;
    last_error_pitch = error;
    PID_pitch_out = Pterm_pitch + Iterm_pitch + Dterm_pitch;
    if (PID_pitch_out > MAX_pitch_output)
        PID_pitch_out = MAX_pitch_output;
    if (PID_pitch_out < -MAX_pitch_output)
        PID_pitch_out = -MAX_pitch_output;

    // roll PID calculations
    error = setpoint_roll + roll_angle * 12.5 - 1500;
    Pterm_roll = error * Kp_roll;
    Iterm_roll += (error + last_error_roll) * Ki_roll * dt / 2;
    Dterm_roll = (error - last_error_roll) * Kd_roll / dt;
    last_error_roll = error;
    PID_roll_out = Pterm_roll + Iterm_roll + Dterm_roll;
    if (PID_roll_out > MAX_roll_output)
        PID_roll_out = MAX_roll_output;
    if (PID_roll_out < -MAX_roll_output)
        PID_roll_out = -MAX_roll_output;

    // yaw PID calculations
    error = (setpoint_yaw + abs_yaw_angle) * 12.5;
    Pterm_yaw = error * Kp_yaw;
    Iterm_yaw += (error + last_error_yaw) * Ki_yaw * dt / 2;
    Dterm_yaw = (error - last_error_yaw) * Kd_yaw / dt;
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
