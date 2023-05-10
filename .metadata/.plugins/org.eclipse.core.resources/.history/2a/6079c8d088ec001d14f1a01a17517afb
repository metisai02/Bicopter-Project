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
extern volatile float Kp_angle_pitch;
extern volatile float Ki_angle_pitch;
extern volatile float Kd_angle_pitch;

extern float Kp_angle_roll;
extern float Ki_angle_roll;
extern float Kd_angle_roll;

extern float Kp_angle_yaw;
extern float Ki_angle_yaw;
extern float Kd_angle_yaw;

extern volatile float Kp_rate_pitch;
extern volatile float Ki_rate_pitch;
extern volatile float Kd_rate_pitch;

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

void pid_calculate(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * dt / 2;

//    if (Iterm > 400)
//        Iterm = 400;
//
//    else if (Iterm < -400)
//        Iterm = -400;

    float Dterm = D * (Error - PrevError) / dt;
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
    error = 0.08 * (roll_rc - 1500) - roll_angle;

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
    pitch_rc = 1500;
    error = 0.08 * (pitch_rc - 1500) - pitch_angle;

    pid_calculate(error, Kp_angle_pitch, Ki_angle_pitch, Kd_angle_pitch, last_error_angle[1], last_Iterm_angle[1]);
    setpoint_rate_pitch = PIDReturn[0];
    last_error_angle[1] = PIDReturn[1];
    last_Iterm_angle[1] = PIDReturn[2];

   // setpoint_rate_pitch = math_constrain(setpoint_rate_pitch, -60, 60);

    // Gioi han ratepitch
	//setpoint_rate_pitch = 0;

    error = setpoint_rate_pitch - pitch_rate;

    pid_calculate(error, Kp_rate_pitch, Ki_rate_pitch, Kd_rate_pitch, last_error_rate[0], last_Iterm_rate[0]);
    PID_out->PID_pitch_out = PIDReturn[0];
    last_error_rate[1] = PIDReturn[1];
    last_Iterm_rate[1] = PIDReturn[2];

    // Gioi han PWM
}
