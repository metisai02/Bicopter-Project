/*
 * pid.c
 *
 *  Created on: Apr 20, 2023
 *      Author: DELL
 */

#include "pid.h"
#include "main.h"

// PID variables declaration
float error;
float Pterm_pitch, Iterm_pitch, Dterm_pitch;
float Pterm_roll, Iterm_roll, Dterm_roll;
float Pterm_yaw, Iterm_yaw, Dterm_yaw;

float last_error_angle[3];
float last_Iterm_angle[3];

float last_error_rate[3];
float last_Iterm_rate[3];

float PID_out[3];

float PIDReturn[3];

// setpoint declaration
float setpoint_roll, setpoint_rate_roll;
float setpoint_pitch, setpoint_rate_pitch;
float setpoint_yaw;

extern float abs_yaw_angle;

#if (TUNING)
extern float Kp_angle_pitch;
extern float Ki_angle_pitch;
extern float Kd_angle_pitch;

extern float Kp_angle_roll;
extern float Ki_angle_roll;
extern float Kd_angle_roll;

extern float Kp_angle_yaw;
extern float Ki_angle_yaw;
extern float Kd_angle_yaw;

extern float Kp_rate_pitch;
extern float Ki_rate_pitch;
extern float Kd_rate_pitch;

extern float Kp_rate_roll;
extern float Ki_rate_roll;
extern float Kd_rate_roll;

extern float Kp_rate_yaw;
extern float Ki_rate_yaw;
extern float Kd_rate_yaw;

#endif

static float math_constrain(float value, float min, float max)
{
    if (value > max)
        value = max;
    else if (value < min)
        value = min;
    return value;
}

//-----------------------------------------------------------------------------
// calculate PID
//-----------------------------------------------------------------------------
// void calculate_PID(uint16_t roll_rc, uint16_t pitch_rc, uint16_t yaw_rc, float roll_angle, float pitch_angle, float yaw_angle, PID_t *PID_out)
// {
//     // set a dead band at zero to improve stability on roll
//     if (roll_rc > 1500 || roll_rc < 1440)
//     {
//         setpoint_roll = roll_rc;
//     }
//     else
//     {
//         setpoint_roll = 1500;
//     }

//     // set a dead band at zero to improve stability on pitch
//     if (pitch_rc > 1540 || pitch_rc < 1440)
//     {
//         setpoint_pitch = pitch_rc;
//     }
//     else
//     {
//         setpoint_pitch = 1500;
//     }

//     // set a dead band at zero to improve stability on yaw
//     if (yaw_rc > 1390)
//     {
//         setpoint_yaw = setpoint_yaw + 0.7;
//     }
//     else if (yaw_rc < 1357)
//     {
//         setpoint_yaw = setpoint_yaw - 0.7;
//     }

//     // pitch PID calculations
//     error = setpoint_pitch - pitch_angle * 12.5 - 1500;
//     Pterm_pitch = error * Kp_pitch;
//     Iterm_pitch += (error + last_error_pitch) * Ki_pitch * dt / 2;
//     Dterm_pitch = (error - last_error_pitch) * Kd_pitch / dt;
//     last_error_pitch = error;
//     PID_pitch_out = Pterm_pitch + Iterm_pitch + Dterm_pitch;
//     if (PID_pitch_out > MAX_pitch_output)
//         PID_pitch_out = MAX_pitch_output;
//     if (PID_pitch_out < -MAX_pitch_output)
//         PID_pitch_out = -MAX_pitch_output;

//     // roll PID calculations
//     error = setpoint_roll - roll_angle * 12.5 - 1500;
//     Pterm_roll = error * Kp_roll;
//     Iterm_roll += (error + last_error_roll) * Ki_roll * dt / 2;
//     Dterm_roll = (error - last_error_roll) * Kd_roll / dt;
//     last_error_roll = error;
//     PID_roll_out = Pterm_roll + Iterm_roll + Dterm_roll;
//     if (PID_roll_out > MAX_roll_output)
//         PID_roll_out = MAX_roll_output;
//     if (PID_roll_out < -MAX_roll_output)
//         PID_roll_out = -MAX_roll_output;

//     // yaw PID calculations
//     error = (setpoint_yaw + abs_yaw_angle) * 12.5;
//     Pterm_yaw = error * Kp_yaw;
//     Iterm_yaw += (error + last_error_yaw) * Ki_yaw * dt / 2;
//     Dterm_yaw = (error - last_error_yaw) * Kd_yaw / dt;
//     last_error_yaw = error;
//     PID_yaw_out = Pterm_yaw + (Iterm_yaw * Ki_yaw) + Dterm_yaw;
//     if (PID_yaw_out > MAX_yaw_output)
//         PID_yaw_out = MAX_yaw_output;
//     if (PID_yaw_out < -MAX_yaw_output)
//         PID_yaw_out = -MAX_yaw_output;

//     PID_out->PID_roll_out = PID_roll_out;
//     PID_out->PID_pitch_out = PID_pitch_out;
//     PID_out->PID_yaw_out = PID_yaw_out;
// }

void pid_calculate(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) / 2;

    if (Iterm > 400)
        Iterm = 400;

    else if (Iterm < -400)
        Iterm = -400;

    float Dterm = D * (Error - PrevError);
    float PIDOutput = Pterm + Iterm + Dterm;

    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}

void pid_roll(uint16_t roll_rc, float roll_angle, float roll_rate, PID_t *PID_out)
{
    if (roll_rc > 1500 || roll_rc < 1440)
    {
        roll_rc = roll_rc;
    }
    else
    {
        roll_rc = 1500;
    }
    error = roll_rc - roll_angle * 12.5 - 1500;

    pid_calculate(error, Kp_angle_roll, Ki_angle_roll, Kd_angle_roll, last_error_angle[0], last_Iterm_angle[0]);
    setpoint_rate_roll = PIDReturn[0];
    last_error_angle[0] = PIDReturn[1];
    last_Iterm_angle[0] = PIDReturn[2];

    // Gioi han rateroll

    error = setpoint_rate_roll - roll_rate;

    pid_calculate(error, Kp_rate_roll, Ki_rate_roll, Kd_rate_roll, last_error_rate[0], last_Iterm_rate[0]);
    PID_out->PID_roll_out = PIDReturn[0];
    last_error_rate[0] = PIDReturn[1];
    last_Iterm_rate[0] = PIDReturn[2];

    // Gioi han PWM
}

void pid_pitch(uint16_t pitch_rc, float pitch_angle, float pitch_rate, PID_t *PID_out)
{
    if (pitch_rc > 1540 || pitch_rc < 1440)
    {
        pitch_rc = pitch_rc;
    }
    else
    {
        pitch_rc = 1500;
    }
    error = pitch_rc - pitch_angle * 12.5 - 1500;

    pid_calculate(error, Kp_angle_pitch, Ki_angle_pitch, Kd_angle_pitch, last_error_angle[1], last_Iterm_angle[1]);
    setpoint_rate_pitch = PIDReturn[0];
    last_error_angle[1] = PIDReturn[1];
    last_Iterm_angle[1] = PIDReturn[2];

    // Gioi han ratepitch
    setpoint_rate_pitch = math_constrain(setpoint_rate_pitch, -75, 75);

    error = setpoint_rate_pitch - pitch_rate;

    pid_calculate(error, Kp_rate_pitch, Ki_rate_pitch, Kd_rate_pitch, last_error_rate[0], last_Iterm_rate[0]);
    PID_out->PID_pitch_out = PIDReturn[0];
    last_error_rate[1] = PIDReturn[1];
    last_Iterm_rate[1] = PIDReturn[2];

    // Gioi han PWM
}
