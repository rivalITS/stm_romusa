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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"

#include "usbd_cdc_if.h"
#include "slider.h"
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

//master slave
uint8_t tx_buff[32]={'i','t','s'};
uint8_t Rx_buff[32];
uint8_t Rx_temp;
uint8_t Rx_state;
uint8_t calib_state;

//Odom
float EncRoll[2];
float RoboPose_Global[3];
float RoboPose[3];
float RoboTwist[3];
float RoboTwist_Global[3];
uint16_t enc_sum[2];

//time
uint32_t time_s;
uint32_t time_stamp_buzz;
uint32_t time_stamp_rx;
//vel
float RobotSpeed[3];


//pc
bool vacuum_status = 0, pneumatic_stat = 0;
bool startButton, retryButton;
bool mapButton;
bool pump_stat;
bool servo_stat;


uint8_t stm_to_pc[84] = {'i', 't', 's'};
uint8_t pc_to_stm[43];

//slider
short int encArm, tarArm, limArm;
short int encLifter, tarLifter, limLifter;

bool reset_arm;
uint32_t cnt_test;

uint32_t time_kirim;

uint8_t status_slider;
uint32_t timer_slider;
short int prev_enc_slider;


uint8_t status_lifter;
uint32_t timer_lifter;
short int prev_enc_lifter;

short int test_pwm;
short int lifter_test;

int sensor;

uint8_t arm_sensor,prev_armsensor;

uint32_t var;



//sensor
bool prox_kanan, prox_kiri;
int16_t IR_kanan, IR_kiri;
uint32_t buff_adc;

uint8_t data_temp;
uint8_t data_buff[35];
uint8_t data_state;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Master_Kirim(void)
{
	  memcpy(tx_buff+3,RobotSpeed,12);
	  uint16_t check_s=0;

	  for(int i = 3;i<30;i++)
	  {
		  check_s += tx_buff[i];
	  }
	  check_s = ~check_s;

	  memcpy(tx_buff+30,&check_s,2);

	  HAL_UART_Transmit_IT(&huart1, tx_buff, 32);
}
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(buzz_GPIO_Port, buzz_Pin, GPIO_PIN_SET);

  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart1, &Rx_temp, 1);
  HAL_UART_Receive_IT(&huart2, &data_temp, 1);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GetTick() - time_s >= 100){
		for(int i = 0;i<3;i++)
		{
			HAL_GPIO_WritePin(buzz_GPIO_Port, buzz_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(buzz_GPIO_Port, buzz_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
		}
		Rx_state = 0;
		HAL_UART_Receive_IT(&huart1, &Rx_temp, 1);
	  }

	  //transmit to pc
	  if(HAL_GPIO_ReadPin(startBut_GPIO_Port, startBut_Pin) == GPIO_PIN_RESET){
		  startButton = 1;
	  }
	  else{
		  startButton = 0;
	  }
	  if(HAL_GPIO_ReadPin(retryBut_GPIO_Port, retryBut_Pin) == GPIO_PIN_RESET){
		  retryButton = 1;
	  }
	  else{
		  retryButton = 0;
	  }


	  if(HAL_GPIO_ReadPin(mapBut_GPIO_Port, mapBut_Pin) == GPIO_PIN_RESET){
		  mapButton = 1;
	  }
	  else{
		  mapButton = 0;
	  }

	  if(HAL_GPIO_ReadPin(prox1_GPIO_Port, prox1_Pin) == GPIO_PIN_RESET){
		  prox_kanan = 1;
	  }
	  else{
		  prox_kanan = 0;
	  }
	  if(HAL_GPIO_ReadPin(prox2_GPIO_Port, prox2_Pin) == GPIO_PIN_RESET){
		  prox_kiri = 1;
	  }
	  else{
		  prox_kiri = 0;
	  }

//	  AVG_Sens = (ADC_Read[0] + ADC_Read[1] + ADC_Read[2] + ADC_Read[3]
//			    + ADC_Read[6] + ADC_Read[7] + ADC_Read[8] + ADC_Read[10]
//				+ ADC_Read[11] + ADC_Read[12] + ADC_Read[13] + ADC_Read[14]
//				+ ADC_Read[15])*0.076923077;
//		  AVG_Sens *= 0.2;

//	  if(AVG_Sens > 2700)
//		  deteksi_sampah = plastik_putih;
//	  else if(AVG_Sens > 1100 && AVG_Sens < 1450)
//		  deteksi_sampah = plastik_biru;
//	  else if(AVG_Sens > 1700 && AVG_Sens < 1900)
//		  deteksi_sampah = daun_kering;
//	  else if(AVG_Sens > 2000 && AVG_Sens < 2400)
//		  deteksi_sampah = kertas_coklat;
//	  else if(AVG_Sens > 500 && AVG_Sens < 800)
//		  deteksi_sampah = daun_hijau;
//	  else
//		  deteksi_sampah = no_detect;
//	  8 2 15 1 14
//	  AVG_Sens = (ADC_Read[1] + ADC_Read[2] + ADC_Read[8] + ADC_Read[14] + ADC_Read[15])*0.2;



	  memcpy(stm_to_pc+3, RoboPose_Global, 12);
	  memcpy(stm_to_pc+15, RoboPose, 8);
	  memcpy(stm_to_pc+23, &time_stamp_rx, 4);
	  memcpy(stm_to_pc+27, &encArm, 2);
	  memcpy(stm_to_pc+29, &encLifter, 2);
	  memcpy(stm_to_pc+31, &startButton, 1);
	  memcpy(stm_to_pc+32, &retryButton, 1);
	  memcpy(stm_to_pc+33, &mapButton, 1);
	  memcpy(stm_to_pc+34, &prox_kanan, 1);
	  memcpy(stm_to_pc+35, &prox_kiri, 1);
	  memcpy(stm_to_pc+36, ADC_Read, 32);


	  uint16_t check_s_pc=0;

	  for(int i = 3; i<82; i++)
	  {
		  check_s_pc += stm_to_pc[i];
	  }
	  check_s_pc = ~check_s_pc;

	  memcpy(stm_to_pc+82, &check_s_pc, 2);


	  CDC_Transmit_FS(stm_to_pc, 84);


	  HAL_Delay(1);


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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Master-slave
	if (huart->Instance == USART1)
	{
		if (Rx_temp == 'i' && Rx_state == 0)
		{
			Rx_state = 1;
			Rx_buff[0] = Rx_temp;
			HAL_UART_Receive_IT(&huart1, &Rx_temp, 1);
		}
		else if (Rx_temp == 't' && Rx_state == 1)
		{
			Rx_state = 2;
			Rx_buff[1] = Rx_temp;
			HAL_UART_Receive_IT(&huart1, &Rx_temp, 1);
		}
		else if (Rx_temp == 's' && Rx_state == 2)
		{
			Rx_state = 3;
			Rx_buff[2] = Rx_temp;
			HAL_UART_Receive_IT(&huart1, Rx_buff + 3, 29);
		}
		else if (Rx_state == 3)
		{
			uint16_t check_s = 0;
			for (int i = 3; i < 30; i++)
			{
				check_s += Rx_buff[i];
			}
			check_s = ~check_s;

			uint16_t check_recv;
			memcpy(&check_recv, Rx_buff + 30, 2);

			if (check_recv == check_s)
			{
				time_s = HAL_GetTick();
				memcpy(&RoboPose_Global[0], Rx_buff + 3, 4);
				memcpy(&RoboPose_Global[1], Rx_buff + 7, 4);
				memcpy(&RoboPose_Global[2], Rx_buff + 11, 4);
				memcpy(&RoboPose[0], Rx_buff + 15, 4);
				memcpy(&RoboPose[1], Rx_buff + 19, 4);
				memcpy(&time_stamp_rx, Rx_buff + 23, 4);

			}

			Rx_state = 0;
			HAL_UART_Receive_IT(&huart1, &Rx_temp, 1);
		}
		else
		{
			Rx_state = 0;
			HAL_UART_Receive_IT(&huart1, &Rx_temp, 1);
		}
	}
	else if(huart->Instance == USART2)
	{
		if (data_temp == 'i' && data_state == 0)
		{
			data_state = 1;
			data_buff[0] = data_temp;
			HAL_UART_Receive_IT(&huart2, &data_temp, 1);
		}
		else if (data_temp == 't' && data_state == 1)
		{
			data_state = 2;
			data_buff[1] = data_temp;
			HAL_UART_Receive_IT(&huart2, &data_temp, 1);
		}
		else if (data_temp == 's' && data_state == 2)
		{
			data_state = 3;
			data_buff[2] = data_temp;
			HAL_UART_Receive_IT(&huart2, data_buff + 3, 32);
		}
		else if (Rx_state == 3)
		{
			memcpy(ADC_Read, data_buff+3, 32);

			data_state = 0;
			HAL_UART_Receive_IT(&huart2, &data_temp, 1);
		}
		else
		{
			data_state = 0;
			HAL_UART_Receive_IT(&huart2, &data_temp, 1);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	// Timer 50 hz
	if(htim->Instance == TIM6)
	{
		encArm = TIM5->CNT;
		encLifter = TIM8->CNT;

	   if(vacuum_status == 1)
		   HAL_GPIO_WritePin(vac_GPIO_Port, vac_Pin, GPIO_PIN_SET);
	   else
		   HAL_GPIO_WritePin(vac_GPIO_Port, vac_Pin, GPIO_PIN_RESET);
	   if(pump_stat == 1)
		   HAL_GPIO_WritePin(mini_pump_GPIO_Port, mini_pump_Pin, GPIO_PIN_SET);
	   else
		   HAL_GPIO_WritePin(mini_pump_GPIO_Port, mini_pump_Pin, GPIO_PIN_RESET);

	   if(pneumatic_stat == 1)
		   HAL_GPIO_WritePin(pneu_GPIO_Port, pneu_Pin, GPIO_PIN_SET);
	   else
		   HAL_GPIO_WritePin(pneu_GPIO_Port, pneu_Pin, GPIO_PIN_RESET);



//	   pwm_Arm(test_pwm);
//	   pwm_Lifter(lifter_test);
	   if (status_lifter == 2)
		{
			if (status_slider == 255)
			{
				Arm(tarArm, encArm, limArm);
			}
			else if (status_slider == 0)
			{
				pwm_Arm(30);
			}
			else if (status_slider == 1)
			{
				Arm(1700, encArm, 30);

				if (encArm > 1500)
					status_slider = 255;
			}


		   if(status_slider == 0)
		   {
			   pwm_Arm(-30);
			   if(++timer_slider > 25)
			   {
				   timer_slider = 0;
				   status_slider = 1;
			   }
			   prev_enc_slider = encArm;
		   }
		   else if(status_slider == 1)
		   {
			   pwm_Arm(-30);
			   if(prev_enc_slider == encArm)
				   timer_slider++;
			   else
				   timer_slider =0;

			   if(timer_slider > 50)
			   {
				   status_slider = 2;
				   TIM5->CNT = 0;
			   }

			   prev_enc_slider = encArm;
		   }
		   else
		   {
			   Arm(tarArm, encArm, limArm);
		   }

		}

	   switch(status_lifter)
	   {
	   case 0:

		   pwm_Lifter(-30);
		   if(++timer_lifter>25)
		   {
			   timer_lifter = 0;

			   status_lifter =1;
		   }

		   prev_enc_lifter = encLifter;
		   break;

	   case 1:


		   pwm_Lifter(-30);

		   if(prev_enc_lifter == encLifter)
		   {
			   timer_lifter++;
		   }
		   else
		   {
			   timer_lifter = 0;
		   }

		   prev_enc_lifter = encLifter;


		   if(timer_lifter > 50)
		   {
			   status_lifter = 2;
			   TIM8->CNT = 0;
		   }
		   break;

	   case 2:

		   Lifter(tarLifter, encLifter, limLifter);

		   break;
	   }

	}

	else if(htim->Instance == TIM7)
	{

		if(++var > 1000){
			RobotSpeed[0] = 0;
			RobotSpeed[1] = 0;
			RobotSpeed[2] = 0;
		}
		Master_Kirim();
	}
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
