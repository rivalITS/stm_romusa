/*
 * slider.h
 *
 *  Created on: Apr 29, 2024
 *      Author: nakanomiku
 */

#ifndef INC_SLIDER_H_
#define INC_SLIDER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdlib.h"


void Arm(short int targetArm, short int encArm, short int limitArm);
void pwm_Arm(short int pwm_outputArm);
void Lifter(short int targetLifter, short int encLifter, short int limitLifter);
void pwm_Lifter(short int pwm_outputLifter);

#ifdef __cplusplus
}
#endif

#endif /* INC_SLIDER_H_ */
