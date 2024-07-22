/*
 * gyroRionTech.h
 *
 *  Created on: May 28, 2024
 *      Author: nakanomiku
 */

#ifndef INC_GYRORIONTECH_H_
#define INC_GYRORIONTECH_H_

#include "main.h"

extern float gyroRaw;
extern int test_reset;
extern int statusGyroo;
extern uint8_t gyroKirim[6];
extern uint32_t tick_gyro;

void init_gyro(UART_HandleTypeDef *pUART);
void gyroUART_handler(UART_HandleTypeDef *pUART);
void gyro_reset(UART_HandleTypeDef *pUART);

#endif /* INC_GYRORIONTECH_H_ */
