/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ADC_Buff[8];
uint16_t ADC_Read[16];

float AVG_Sens;

typedef enum{
	plastik_putih,
	plastik_biru,
	kertas_coklat,
	kertas_koran,
	daun_hijau,
	daun_kering,
	botol_plastik,
	fero,
	non_fero,
	no_detect

}Sampah;


Sampah deteksi_sampah;

uint8_t test_uart[6]={'R','I','V','A','L','\n'};

uint8_t data_uart_send[35] = {'i','t','s'};
uint16_t led_bright=3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart_send(uint8_t data);
void uart_ex_init(void);
void delay_micro(uint32_t time);
uint8_t soft_pwm(uint16_t Duty);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_ALL);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_ALL);
//  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim11);


//  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 250;
//  TIM2->CCR1 = TIM2->CCR2 = 250;
//  TIM3->CCR4 = TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = 250;
//  TIM4->CCR1 = TIM4->CCR2 = 250;

  HAL_ADC_Start_DMA(&hadc1, ADC_Buff, 16);

  uart_ex_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  ADC_Read[0] = ADC_Buff[0] & 0xFFFF;
	  ADC_Read[1] = ADC_Buff[0] >> 16;
	  ADC_Read[2] = ADC_Buff[1] & 0xFFFF;
	  ADC_Read[3] = ADC_Buff[1] >> 16;
	  ADC_Read[4] = ADC_Buff[2] & 0xFFFF;
	  ADC_Read[5] = ADC_Buff[2] >> 16;
	  ADC_Read[6] = ADC_Buff[3] & 0xFFFF;
	  ADC_Read[7] = ADC_Buff[3] >> 16;
	  ADC_Read[8] = ADC_Buff[4] & 0xFFFF;
	  ADC_Read[9] = ADC_Buff[4] >> 16;
	  ADC_Read[10] = ADC_Buff[5] & 0xFFFF;
	  ADC_Read[11] = ADC_Buff[5] >> 16;
	  ADC_Read[12] = ADC_Buff[6] & 0xFFFF;
	  ADC_Read[13] = ADC_Buff[6] >> 16;
	  ADC_Read[14] = ADC_Buff[7] & 0xFFFF;
	  ADC_Read[15] = ADC_Buff[7] >> 16;

	  memcpy(data_uart_send+3,ADC_Read,sizeof(ADC_Read));
	  for(int i=0;i<35;i++)
	  {
		  uart_send(data_uart_send[i]);
	  }
	  HAL_Delay(10);

	  AVG_Sens = ADC_Read[0] + ADC_Read[1] + ADC_Read[2] + ADC_Read[3] + ADC_Read[6] + ADC_Read[7] + ADC_Read[8] + ADC_Read[9] + ADC_Read[10] + ADC_Read[11] + ADC_Read[12] + ADC_Read[13] + ADC_Read[14] + ADC_Read[15];
	  AVG_Sens *= 0.2;

	  if(AVG_Sens > 2700)
		  deteksi_sampah = plastik_putih;
	  else if(AVG_Sens > 1100 && AVG_Sens < 1450)
		  deteksi_sampah = plastik_biru;
	  else if(AVG_Sens > 1700 && AVG_Sens < 1900)
		  deteksi_sampah = daun_kering;
	  else if(AVG_Sens > 2000 && AVG_Sens < 2400)
		  deteksi_sampah = kertas_coklat;
	  else if(AVG_Sens > 500 && AVG_Sens < 800)
		  deteksi_sampah = daun_hijau;
	  else
		  deteksi_sampah = no_detect;






  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

if(htim->Instance == TIM11)
{
//	HAL_GPIO_TogglePin(TIM_EX1_GPIO_Port,TIM_EX1_Pin);
//	HAL_GPIO_TogglePin(TIM_EX2_GPIO_Port,TIM_EX2_Pin);

	 uint8_t out_pwm;
	 out_pwm = soft_pwm(led_bright);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,out_pwm);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,out_pwm);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,out_pwm);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,out_pwm);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,out_pwm);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,out_pwm);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,out_pwm);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,out_pwm);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,out_pwm);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,out_pwm);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,out_pwm);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,out_pwm);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,out_pwm);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,out_pwm);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,out_pwm);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,out_pwm);

//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
}

}



void delay_micro(uint32_t time)
{
	TIM10->CNT = 0;
	while(TIM10->CNT < time);
}

void uart_ex_init(void)
{
	HAL_GPIO_WritePin(UART_S_GPIO_Port, UART_S_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start(&htim10);

}

void uart_send(uint8_t data)
{

	//Send Start BIT+
	HAL_GPIO_WritePin(UART_S_GPIO_Port, UART_S_Pin, GPIO_PIN_RESET);
	delay_micro(52);

	//Send Data
	for(int i= 0; i<8; i++)
	{
		HAL_GPIO_WritePin(UART_S_GPIO_Port, UART_S_Pin, (data >> i) & 0x01);
		delay_micro(52);
	}

	//Send Stop BIT
	HAL_GPIO_WritePin(UART_S_GPIO_Port, UART_S_Pin, GPIO_PIN_SET);
	delay_micro(52);

}


uint8_t soft_pwm(uint16_t Duty)
{
	uint8_t out;
	static uint16_t cnt_ccr;

	if(cnt_ccr < Duty)
		out = 1;
	else
		out = 0;

	if(++cnt_ccr > 10)
		cnt_ccr = 0;

	return out;


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
