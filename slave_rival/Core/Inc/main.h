/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define lcd_anode_Pin GPIO_PIN_2
#define lcd_anode_GPIO_Port GPIOE
#define lcd_db5_Pin GPIO_PIN_3
#define lcd_db5_GPIO_Port GPIOE
#define lcd_db4_Pin GPIO_PIN_4
#define lcd_db4_GPIO_Port GPIOE
#define M3_PWM_Pin GPIO_PIN_5
#define M3_PWM_GPIO_Port GPIOE
#define lcd_rs_Pin GPIO_PIN_13
#define lcd_rs_GPIO_Port GPIOC
#define lcd_out_Pin GPIO_PIN_14
#define lcd_out_GPIO_Port GPIOC
#define lcd_e_Pin GPIO_PIN_15
#define lcd_e_GPIO_Port GPIOC
#define ODOM1_A_Pin GPIO_PIN_0
#define ODOM1_A_GPIO_Port GPIOA
#define ODOM1_B_Pin GPIO_PIN_1
#define ODOM1_B_GPIO_Port GPIOA
#define ENC0_A_Pin GPIO_PIN_9
#define ENC0_A_GPIO_Port GPIOE
#define M3_D2_Pin GPIO_PIN_10
#define M3_D2_GPIO_Port GPIOE
#define ENC0_B_Pin GPIO_PIN_11
#define ENC0_B_GPIO_Port GPIOE
#define M2_D1_Pin GPIO_PIN_12
#define M2_D1_GPIO_Port GPIOE
#define M3_D1_Pin GPIO_PIN_13
#define M3_D1_GPIO_Port GPIOE
#define M1_D2_Pin GPIO_PIN_14
#define M1_D2_GPIO_Port GPIOE
#define M2_D2_Pin GPIO_PIN_15
#define M2_D2_GPIO_Port GPIOE
#define M1_D1_Pin GPIO_PIN_11
#define M1_D1_GPIO_Port GPIOB
#define M0_D1_Pin GPIO_PIN_12
#define M0_D1_GPIO_Port GPIOB
#define M0_D2_Pin GPIO_PIN_13
#define M0_D2_GPIO_Port GPIOB
#define M1_PWM_Pin GPIO_PIN_14
#define M1_PWM_GPIO_Port GPIOB
#define M0_PWM_Pin GPIO_PIN_15
#define M0_PWM_GPIO_Port GPIOB
#define Buzz_Pin GPIO_PIN_9
#define Buzz_GPIO_Port GPIOD
#define ODOM0_A_Pin GPIO_PIN_6
#define ODOM0_A_GPIO_Port GPIOC
#define ODOM0_B_Pin GPIO_PIN_7
#define ODOM0_B_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_15
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_3
#define ENC1_B_GPIO_Port GPIOB
#define ENC2_A_Pin GPIO_PIN_4
#define ENC2_A_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_5
#define ENC2_B_GPIO_Port GPIOB
#define ENC3_A_Pin GPIO_PIN_6
#define ENC3_A_GPIO_Port GPIOB
#define ENC3_B_Pin GPIO_PIN_7
#define ENC3_B_GPIO_Port GPIOB
#define lcd_db7_Pin GPIO_PIN_0
#define lcd_db7_GPIO_Port GPIOE
#define lcd_db6_Pin GPIO_PIN_1
#define lcd_db6_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
