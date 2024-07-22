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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "lcd.h"
#include "motor.h"
#include "gyroRionTech.h"
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

// Master-Slave
uint8_t rx_state;
uint8_t rx_temp;
uint8_t rx_buff[32];
uint8_t tx_buff[32]={'i','t','s'};

uint8_t MasterState;
uint32_t MasterTimeout;


// Variabel Motor
int16_t RotEnc[2];
int16_t pwm[4];
int16_t WheelSpeed_ENC[4];

extern float WheelSpeed_SI[4];
extern float WheelSpeed_SP[4];
extern float RobotSpeed[3];

//Uart Gyro
uint8_t uart2_rxData;
uint8_t uart_gyroStatus;
uint8_t uart_gyroData[8];
int16_t buffGyro[3];
float GyroRoll,GyroPitch,GyroYaw;
uint8_t statusGyro = 0;
float offest_gyro;
float LastGyro;
uint8_t GyroReady;


//Odom
float EncRoll[2];
float RoboPose_Global[3];
float RoboPose[3];
float RoboTwist[3];
float RoboTwist_Global[3];
uint16_t enc_sum[2];


//Buzz;
uint32_t time_s;
uint32_t time_stamp;
uint32_t diff_stamp;
uint32_t time_stamp_tx;

char lcd[64];

// Test Variable
short int pwm_test[4];
uint32_t count;

uint32_t time_send;
uint32_t sampling_gyro;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Slave_Kirim(void)
{
	  //Send To Master
	  time_stamp_tx = HAL_GetTick();
	  memcpy(tx_buff+3,&RoboPose_Global[0],4);
	  memcpy(tx_buff+7,&RoboPose_Global[1],4);
	  memcpy(tx_buff+11,&RoboPose_Global[2],4);
	  memcpy(tx_buff+15,&RoboPose[0],4);
	  memcpy(tx_buff+19,&RoboPose[1],4);
	  memcpy(tx_buff+23, &time_stamp_tx, 4);

	  uint16_t check_s=0;

	  for(int i = 3; i<30; i++){
		  check_s += tx_buff[i];
	  }
	  check_s = ~check_s;

	  memcpy(tx_buff+30, &check_s, 2);

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
  MX_DMA_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//  HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_SET);

  //Init Encoder
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  //Init Motor
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);



  //Init Master slave
  HAL_UART_Receive_IT(&huart1, &rx_temp, 1);


  // Inint Gyro
//  HAL_UART_Receive_IT(&huart2, &uart2_rxData, 1);
  init_gyro(&huart2);

  //Start Interrupt
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  lcd_init(16, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  lcd_print(0, 0, "hai");
	  sprintf(lcd, "X=%.2f", RoboPose_Global[0]);
	  lcd_print(0, 0, lcd);
	  sprintf(lcd, "Y=%.2f", RoboPose_Global[1]);
	  lcd_print(8, 0, lcd);
//	  sprintf(lcd, "G=%.2f", RoboPose_Global[2]);
	  sprintf(lcd, "G=%.2f || %d", RoboPose_Global[2],GyroReady);
	  lcd_print(0, 1, lcd);
	  sprintf(lcd, "x=%.2f", RobotSpeed[0]);
	  lcd_print(0, 2, lcd);
	  sprintf(lcd, "y=%.2f", RobotSpeed[1]);
	  lcd_print(8, 2, lcd);
	  sprintf(lcd, "w=%.2f", RobotSpeed[2]);
	  lcd_print(0, 3, lcd);
	  sprintf(lcd, "%c", rx_buff[0]);
	  lcd_print(8, 3, lcd);
	  sprintf(lcd, "%c", rx_buff[1]);
	  lcd_print(9, 3, lcd);
	  sprintf(lcd, "%c", rx_buff[2]);
	  lcd_print(10, 3, lcd);
//	  sprintf(lcd, "%.3d", (HAL_GetTick() - time_stamp)/1000);
//	  lcd_print(11, 3, lcd);
	  diff_stamp = HAL_GetTick() - time_stamp;


	  if(HAL_GetTick() - time_stamp >= 100){
		lcd_print(11, 3, " pts");
//		lcd_clear();
		rx_state = 0;
		HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
	  }
	  else{
		  lcd_print(11, 3, "     ");
	  }


	  if(HAL_GetTick() - tick_gyro > 100)
	  {
		  init_gyro(&huart2);
	  }


//
//	  if(statusGyro == 1)
//	  {
//
//		  if(++sampling_gyro > 100)
//		  {
////			if((fabs(LastGyro - GyroYaw) < 0.01 ) && (HAL_GetTick() > 100))
//			if((fabs(LastGyro - GyroYaw) < 0.01))
//				GyroReady = 1;
//
//			LastGyro = GyroYaw;
//
//			sampling_gyro = 0;
//
//		  }
//
//	  }


//	  if(HAL_GetTick() - time_s > 2000)
//	  {
//		  time_s = HAL_GetTick();
//
//		  for(int i = 0;i<3;i++)
//		  {
//			  HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);
//			  HAL_Delay(50);
//			  HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_SET);
//			  HAL_Delay(50);
//		  }
//	  }

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
	if(huart->Instance == USART1)
	{
		if(rx_temp == 'i' && rx_state == 0)
		{
			rx_state = 1;
			rx_buff[0] = rx_temp;
			HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
		}
		else if(rx_temp == 't' && rx_state == 1)
		{
			rx_state = 2;
			rx_buff[1] = rx_temp;
			HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
		}
		else if(rx_temp == 's' && rx_state == 2)
		{
			rx_state = 3;
			rx_buff[2] = rx_temp;
			HAL_UART_Receive_IT(&huart1, rx_buff+3, 29);
		}
		else if(rx_state == 3)
		{
			uint16_t check_s=0;
			for(int i=3;i<30;i++)
			{
				check_s += rx_buff[i];
			}
			check_s = ~check_s;

			uint16_t check_recv;
			memcpy(&check_recv,rx_buff+30,2);

			if(check_recv == check_s)
			{
				time_stamp = HAL_GetTick();
//				memcpy(&count, rx_buff+15, sizeof(count));
				MasterState =1;
				MasterTimeout=0;
				memcpy(RobotSpeed,rx_buff+3,12);

			}

			rx_state = 0;
			HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
		}
		else
		{
			rx_state = 0;
			HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
		}
	}

	//GyroRionTech
	else if(huart->Instance ==  USART2)
	{
		gyroUART_handler(&huart2);
		GyroYaw = gyroRaw - offest_gyro;
		if(GyroYaw > 0 || GyroYaw <0){
			GyroReady = 1;
		}

		if(GyroYaw > 180)
			GyroYaw -= 360;
		else if(GyroYaw < -180)
			GyroYaw += 360;

		if(statusGyroo < 4)
		{
			offest_gyro = gyroRaw;
		}
	}

	//Gyro
//	else if(huart->Instance == USART2)
//	{
//		if (uart2_rxData == 0xAA && uart_gyroStatus == 0)
//		{
//			HAL_UART_Receive_IT(&huart2, uart_gyroData + 1, 7);
//			uart_gyroData[0] = 0xAA;
//			uart_gyroStatus = 1;
//		}
//		else if (uart_gyroStatus == 1)
//		{
//
//			if (uart_gyroData[0] == 0xAA && uart_gyroData[7] == 0x55)
//			{
//
//				buffGyro[0] = (((uint16_t) uart_gyroData[1] << 8)
//						| (uint16_t) uart_gyroData[2]);
//				buffGyro[1] = (((uint16_t) uart_gyroData[3] << 8)
//						| (uint16_t) uart_gyroData[4]);
//				buffGyro[2] = (((uint16_t) uart_gyroData[5] << 8)
//						| (uint16_t) uart_gyroData[6]);
//				float buff_yaw;
//
//				buff_yaw = (float) buffGyro[0] * 0.01;
//
//				if (statusGyro == 0)
//				{
//					statusGyro = 1;
//					offest_gyro = buff_yaw;
//				}
//
//				GyroYaw = buff_yaw - offest_gyro;
//
//				if (GyroYaw > 180)
//					GyroYaw -= 360;
//				else if (GyroYaw < -180)
//					GyroYaw += 360;
//
//				GyroPitch = (float) buffGyro[1] * 0.01;
//				GyroRoll = (float) buffGyro[2] * 0.01;
//
//
//
//
//
//			}
//
//			HAL_UART_Receive_IT(&huart2, &uart2_rxData, 1);
//			uart_gyroStatus = 0;
//
//		}
//		else
//		{
//			HAL_UART_Receive_IT(&huart2, &uart2_rxData, 1);
//			uart_gyroStatus = 0;
//		}
//	}



}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	// Timer 50 hz
	if(htim->Instance == TIM6)
	{

		  WheelSpeed_ENC[0] = __HAL_TIM_GET_COUNTER(&htim1);
		  WheelSpeed_ENC[1] = __HAL_TIM_GET_COUNTER(&htim2);
		  WheelSpeed_ENC[2] = __HAL_TIM_GET_COUNTER(&htim3);
		  WheelSpeed_ENC[3] = __HAL_TIM_GET_COUNTER(&htim4);

		  for( int i = 0; i<4; i++)
		  {
			  WheelSpeed_SI[i] = ((float)WheelSpeed_ENC[i] / 532.0) * M_PI * 2.0 * 50.0;
		  }
		   __HAL_TIM_GET_COUNTER(&htim1) = 0;
		   __HAL_TIM_GET_COUNTER(&htim2) = 0;
		   __HAL_TIM_GET_COUNTER(&htim3) = 0;
		   __HAL_TIM_GET_COUNTER(&htim4) = 0;

		omniKine(RobotSpeed[0], RobotSpeed[1], RobotSpeed[2]);

		if(MasterState == 1)
		{
			MotorSetSpeed(WheelSpeed_SP);

			if(++MasterTimeout > 50)
				MasterState = 0;
		}
		else
		{
			WheelSpeed_SP[0] = 0;
			WheelSpeed_SP[1] = 0;
			WheelSpeed_SP[2] = 0;
			WheelSpeed_SP[3] = 0;
			MotorSetSpeed(WheelSpeed_SP);
//			rx_state = 0;
//			HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
		}




	}

	//Timer 1KHz
	else if(htim->Instance == TIM7)
	{


		if(++time_send>10)
		{
			Slave_Kirim();
			time_send = 0;
		}

		RotEnc[0] = __HAL_TIM_GET_COUNTER(&htim5);
		RotEnc[1] = __HAL_TIM_GET_COUNTER(&htim8);
		enc_sum[0] += RotEnc[0];
		enc_sum[1] += RotEnc[1];

		EncRoll[0] = RotEnc[0] / 1440.0;
		EncRoll[1] = RotEnc[1] / 1440.0;

		RoboTwist[0] = EncRoll[0] * 0.06 * M_PI;
		RoboTwist[1] = EncRoll[1] * 0.06 * M_PI;

		RoboTwist_Global[0] = cosf(GyroYaw * 0.0174533) * RoboTwist[0]
				- sinf(GyroYaw * 0.0174533) * RoboTwist[1];
		RoboTwist_Global[1] = sinf(GyroYaw * 0.0174533) * RoboTwist[0]
				+ cosf(GyroYaw * 0.0174533) * RoboTwist[1];

		RoboPose[0] += RoboTwist[0];
		RoboPose[1] += RoboTwist[1];
		RoboPose[2] = GyroYaw * 0.0174533;

		RoboPose_Global[0] += RoboTwist_Global[0]; //dalam meter
		RoboPose_Global[1] += RoboTwist_Global[1]; //jdi kebacanya 0.05m berrti 50 cm
		RoboPose_Global[2] = GyroYaw * 0.0174533;

		   __HAL_TIM_GET_COUNTER(&htim5) = 0;
		   __HAL_TIM_GET_COUNTER(&htim8) = 0;


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
