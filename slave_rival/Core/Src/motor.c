/*
 * motor.c
 *
 *  Created on: Feb 24, 2024
 *      Author: noval
 */

#include "main.h"
#include "motor.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"

extern int16_t pwm[4];
extern int16_t WheelSpeed_ENC[4];
float WheelSpeed_SI[4];
float WheelSpeed_SP[4];
float omni_R = 0.0635;

// PID Motor
float propo[4],integral[4], derivative[4];
float error_motor[4], sum_error[4];
float previous_error_motor[4];
float PID_out[4];
float kp=16,ki=1.5,kd=0;

float RobotSpeed[3];


void MotorSetSpeed(float speedSP[4])
{
	int16_t pwm_[4];

	for(int i=0; i<4; i++)
	{

		if(speedSP[i] == 0)
		{
			sum_error[i] = 0;
		}

		error_motor[i] = speedSP[i] - WheelSpeed_SI[i];
		sum_error[i] += error_motor[i];

		if(sum_error[i] > 499)
			sum_error[i] = 499;
		else if(sum_error[i] < -499)
			sum_error[i] = -499;

		if(speedSP[i]==0)
		{
			sum_error[i] = 0;
			integral[i] =0;
		}

		propo[i] = kp * error_motor[i];
		integral[i] = ki * sum_error[i];
//		derivative[i] = kd*(WheelSpeed_SI[i] - previous_error_motor [i]);

//		previous_error_motor[i] = WheelSpeed_SI[i];
		if(integral[i] > 499)
			integral[i] = 499;
		else if(integral[i] < -499)
			integral[i] = -499;

//		PID_out[i] = propo[i] + integral[i] + derivative[i];
		PID_out[i] = propo[i] + integral[i];

		if(PID_out[i] > 499)
			PID_out[i] = 499;
		else if(PID_out[i] < -499)
			PID_out[i] = -499;

		pwm_[i] = (int16_t)PID_out[i];

	}
	MotorSetPWM(pwm_);
}


void omniKine(float vx, float vy, float vz){

	float wheel_deg[4] = {135,225,315,405};

	float wheel_cos[4];
	float wheel_sin[4];

	for(int i=0;i<4;i++)
	{
		wheel_cos[i] = cosf(wheel_deg[i] * 0.0174533);
		wheel_sin[i] = sinf(wheel_deg[i] * 0.0174533);
	}


	WheelSpeed_SP[0] = (1/omni_R) * (wheel_sin[0]*-vx + wheel_cos[0]*-vy + omni_R*vz);
	WheelSpeed_SP[1] = (1/omni_R) * (wheel_sin[1]*-vx + wheel_cos[1]*-vy + omni_R*vz);
	WheelSpeed_SP[2] = (1/omni_R) * (wheel_sin[2]*-vx + wheel_cos[2]*-vy + omni_R*vz);
	WheelSpeed_SP[3] = (1/omni_R) * (wheel_sin[3]*-vx + wheel_cos[3]*-vy + omni_R*vz);

}


//set PWM motor
void MotorSetPWM(int16_t pwm[4])
{

	// Motor 0
	if(pwm[0] > 0)
	{
		  HAL_GPIO_WritePin(M0_D1_GPIO_Port, M0_D1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(M0_D2_GPIO_Port, M0_D2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		  HAL_GPIO_WritePin(M0_D1_GPIO_Port, M0_D1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(M0_D2_GPIO_Port, M0_D2_Pin, GPIO_PIN_SET);
	}


	// Motor 1
	if(pwm[1] > 0)
	{
		  HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		  HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, GPIO_PIN_SET);
	}


	// Motor 2
	if(pwm[2] > 0)
	{
		  HAL_GPIO_WritePin(M2_D1_GPIO_Port, M2_D1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(M2_D2_GPIO_Port, M2_D2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		  HAL_GPIO_WritePin(M2_D1_GPIO_Port, M2_D1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(M2_D2_GPIO_Port, M2_D2_Pin, GPIO_PIN_SET);
	}

	// Motor 3
	if(pwm[3] < 0)
	{
		  HAL_GPIO_WritePin(M3_D1_GPIO_Port, M3_D1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(M3_D2_GPIO_Port, M3_D2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		  HAL_GPIO_WritePin(M3_D1_GPIO_Port, M3_D1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(M3_D2_GPIO_Port, M3_D2_Pin, GPIO_PIN_SET);
	}



	for(int i=0;i<4;i++)
	{
	 if(pwm[i] > 499)
		 pwm[i] = 499;
	 else if(pwm[i] <-499)
		 pwm[i] = -499;

	}

	TIM12->CCR2 = abs(pwm[0]);
	TIM12->CCR1 = abs(pwm[1]);
	TIM10->CCR1 = abs(pwm[2]);
	TIM9->CCR1 = abs(pwm[3]);
}
