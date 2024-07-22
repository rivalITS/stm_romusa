/*
 * slider.c
 *
 *  Created on: Apr 29, 2024
 *      Author: nakanomiku
 */

#include "slider.h"
#include "math.h"
#include "stdlib.h"

extern short int encArm;
extern short int encLifter;

static float propoArm, integralArm, derivativeArm;
static float propoLifter, integralLifter, derivativeLifter;
static int errorArm, prev_errorArm, sum_errorArm;
static int errorLifter, prev_errorLifter, sum_errorLifter;
short int outputArm;
short int outputLifter;

float kp_Arm=3.0, ki_Arm=0.0, kd_Arm=0.0;
float kp_Lifter=4.0, ki_Lifter=0.0, kd_Lifter=0.0;

void Arm(short int targetArm, short int encArm, short int limitArm)
{
//	static float proportional, integral, derivative;
//	static int error, prev_error, sum_error;
//	short int output;

	if(targetArm < -5)
		targetArm = -5;

//	else if(targetArm>2200)
//		targetArm = 2380;

//	if(limitArm > 50){
//		limitArm = 50;
//	}

	errorArm = targetArm - encArm;
	sum_errorArm += errorArm;

	propoArm			= kp_Arm * errorArm;
	integralArm 		= ki_Arm * sum_errorArm;
	derivativeArm		= kd_Arm * (errorArm - prev_errorArm);

	if(errorArm > 99)
		errorArm = 99;
	else if(errorArm < -99)
		errorArm = -99;

	if(sum_errorArm > 99)
			sum_errorArm = 99;
		else if(sum_errorArm < -99)
			sum_errorArm = 99;

	if (integralArm > 99)
		integralArm = 99;
	else if (integralArm < -99)
		integralArm = -99;
//	baca_integral = integral;
	prev_errorArm = errorArm;

	outputArm = (short int) (propoArm + integralArm + derivativeArm);

	if (outputArm > limitArm)
		outputArm = limitArm;
	else if (outputArm < -1*limitArm)
		outputArm = -1*limitArm;

	pwm_Arm(outputArm);
}

void pwm_Arm(short int pwm_outputArm)
{
	if (pwm_outputArm > 0)
	{

		  HAL_GPIO_WritePin(ARM_D1_GPIO_Port, ARM_D1_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ARM_D2_GPIO_Port, ARM_D2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Motor_s_bak_GPIO_Port, Motor_s_bak_Pin, GPIO_PIN_RESET);
	}
	else
	{
		  HAL_GPIO_WritePin(ARM_D1_GPIO_Port, ARM_D1_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(ARM_D2_GPIO_Port, ARM_D2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Motor_s_bak_GPIO_Port, Motor_s_bak_Pin, GPIO_PIN_SET);

	}
	TIM12->CCR2 = abs(pwm_outputArm);
}

void Lifter(short int targetLifter, short int encLifter, short int limitLifter)
{
//	static float proportional, integral, derivative;
//	static int error, prev_error, sum_error;
//	short int output;
	if(targetLifter<-5){
		targetLifter = -5;
	}
	else if(targetLifter>800){
		targetLifter = 800;
	}

//	if(limitLifter > 30)
//		limitLifter = 30;

	errorLifter = targetLifter - encLifter;
	sum_errorLifter += errorLifter;

	propoLifter			= kp_Lifter * errorLifter;
	integralLifter 		= ki_Lifter * sum_errorLifter;
	derivativeLifter		= kd_Lifter * (errorLifter - prev_errorLifter);

	if(errorLifter > 99)
		errorLifter = 99;
	else if(errorLifter < -99)
		errorLifter = -99;

	if(sum_errorLifter > 99)
			sum_errorLifter = 99;
		else if(sum_errorLifter < -99)
			sum_errorLifter = 99;

	if (integralLifter > 99)
		integralLifter = 99;
	else if (integralLifter < -99)
		integralLifter = -99;
//	baca_integral = integral;
	prev_errorLifter = errorLifter;

	outputLifter = (short int) (propoLifter + integralLifter + derivativeLifter);

	if (outputLifter > limitLifter)
		outputLifter = limitLifter;
	else if (outputLifter < -1*limitLifter)
		outputLifter = -1*limitLifter;

	pwm_Lifter(outputLifter);
}

void pwm_Lifter(short int pwm_outputLifter)
{
	if (pwm_outputLifter > 0)
	{
		  HAL_GPIO_WritePin(LIFT_D1_GPIO_Port, LIFT_D1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LIFT_D2_GPIO_Port, LIFT_D2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		  HAL_GPIO_WritePin(LIFT_D1_GPIO_Port, LIFT_D1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LIFT_D2_GPIO_Port, LIFT_D2_Pin, GPIO_PIN_SET);
	}
	TIM9->CCR1 = abs(pwm_outputLifter);
}
