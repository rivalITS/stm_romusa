/*
 * lcd.h
 *
 *  Created on: Mar 21, 2024
 *      Author: myami
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx.h"
#include "main.h"


#define USE_RW

#define lcd_rs_GPIO_Port GPIOC
#define lcd_rs_Pin GPIO_PIN_13


#ifdef USE_RW

#define lcd_rw_GPIO_Port GPIOC
#define lcd_rw_Pin  GPIO_PIN_14

#endif


#define lcd_e_GPIO_Port GPIOC
#define lcd_e_Pin GPIO_PIN_15

#define lcd_db4_GPIO_Port GPIOE
#define lcd_db4_Pin GPIO_PIN_4

#define lcd_db5_GPIO_Port GPIOE
#define lcd_db5_Pin GPIO_PIN_3

#define lcd_db6_GPIO_Port GPIOE
#define lcd_db6_Pin GPIO_PIN_1

#define lcd_db7_GPIO_Port GPIOE
#define lcd_db7_Pin GPIO_PIN_0

#define lcd_rs_set HAL_GPIO_WritePin(lcd_rs_GPIO_Port,lcd_rs_Pin,GPIO_PIN_SET);
#define lcd_rs_reset HAL_GPIO_WritePin(lcd_rs_GPIO_Port,lcd_rs_Pin,GPIO_PIN_RESET);

#define lcd_e_set HAL_GPIO_WritePin(lcd_e_GPIO_Port,lcd_e_Pin,GPIO_PIN_SET);
#define lcd_e_reset HAL_GPIO_WritePin(lcd_e_GPIO_Port,lcd_e_Pin,GPIO_PIN_RESET);

void lcd_init(uint8_t column, uint8_t row);
void lcd_gotoxy(uint8_t column, uint8_t row);
void lcd_clear(void);
void lcd_putc(uint8_t data);
void lcd_print(uint8_t column, uint8_t row, char* data);
void lcd_4bit(uint8_t data);
void lcd_8bit(uint8_t data);


#ifdef __cplusplus
}
#endif

#endif /* INC_LCD_H_ */
