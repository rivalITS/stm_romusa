/*
 * gyroRionTech.c
 *
 *  Created on: May 28, 2024
 *      Author: nakanomiku
 */


/*
 * gyroRionTech.c
 *
 *  Created on: Aug 17, 2021
 *      Author: itsrobocon
 */

#include "gyroRionTech.h"
#include "usart.h"
#include "stdlib.h"

uint8_t gyroKirim[6]={0x68,0x0d,0x00,0x84,0x10,0x1};
uint8_t gyroTerima[16];

uint8_t buffRX_t;

char statRX_t;

int it_data;

int sign;
float a,b,c,d,e;
float gyroRaw;

int test_reset;
extern float offsetSudutt;
int statusGyroo=0;

uint32_t tick_gyro;


void init_gyro(UART_HandleTypeDef *pUART)
{

	HAL_UART_Receive_DMA(pUART,&buffRX_t,1);
	HAL_UART_Transmit(pUART, gyroKirim, 6,1000);


//	HAL_UART_Transmit_DMA(pUART, gyroKirim, 5);
//	HAL_UART_Receive_DMA(pUART,gyroTerima , 12);

}

void gyroUART_handler(UART_HandleTypeDef *pUART)
{



	if(statRX_t==0 && buffRX_t==0x68)
	{
		gyroTerima[0]=0x68;
		gyroTerima[it_data+1]=buffRX_t;
		statRX_t=1;
		it_data++;
	}
	else if(statRX_t==1)
	{
		gyroTerima[it_data]=buffRX_t;
		if(++it_data>15)
		{
			it_data=0;
			statRX_t=0;

//			if(gyroTerima[0]==0x68 && gyroTerima[1]==0x1f && gyroTerima[2]==0x00 && gyroTerima[3]==0x84)		//punya AR
			if(gyroTerima[0]==0x68 && gyroTerima[1]==0x0d && gyroTerima[2]==0x00 && gyroTerima[3]==0x84)		//punya TR

			{
				sign = gyroTerima[10]/16;
				if(sign == 1)
					sign = -1;
				else
					sign = 1;

				a = gyroTerima[10]%16;
				b = gyroTerima[11]/16%16;
				c = gyroTerima[11]%16;
				d = gyroTerima[12]/16%16;
				e = gyroTerima[12]%16;

				gyroRaw = (float)sign*(a*100 + b*10 + c*1 + d *0.1 + e*0.01) ;

				tick_gyro = HAL_GetTick();

				if(statusGyroo<4)
				{
//					offsetSudutt = gyroRaw;
					statusGyroo+=1;
				}
			}
		}
	}


	HAL_UART_Receive_DMA(pUART,&buffRX_t,1);

}

void gyro_reset(UART_HandleTypeDef *pUART)
{
	uint8_t kirim_rst[5]={0x68, 0x04, 0x00, 0x28, 0x2C};

	HAL_UART_Transmit_DMA(pUART, kirim_rst, 5);

}

