/*
 * motor.h
 *
 *  Created on: Feb 24, 2024
 *      Author: noval
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

extern float Wheel_R;
extern float lx;
extern float ly;


void MotorSetSpeed(float speedSP[4]);
void MotorSetPWM(int16_t pwm[4]);
void omniKine(float vx, float vy, float vz);

#endif /* INC_MOTOR_H_ */
