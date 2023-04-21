/*
 * motor.h
 *
 *  Created on: Apr 21, 2023
 *      Author: NHHanh
 */

#ifndef MOTOR_H_
#define MOTOR_H_

void calculate_motor_output(uint16_t *esc_right, uint16_t *esc_left, uint16_t *servo_right, uint16_t *servo_left,  uint16_t throttle_rc, PID_t pid);

#endif /* MOTOR_H_ */
