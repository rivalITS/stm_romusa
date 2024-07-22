/*
 * lcd.c
 *
 *  Created on: May 26, 2024
 *      Author: nakanomiku
 */

/*
 * lcd.c
 *
 *  Created on: Mar 21, 2024
 *      Author: myami
 */

//From Lamda_Revalls
#include "lcd.h"


uint8_t lcd_address[4];
uint8_t lcd_max_column;
uint8_t lcd_max_row;
uint8_t lcd2_address[4];
uint8_t lcd2_max_column;
uint8_t lcd2_max_row;

int i,wait=650;

void lcd_init(uint8_t column, uint8_t row)
{
#ifdef USE_RW

	HAL_GPIO_WritePin(lcd_rw_GPIO_Port, lcd_rw_Pin, GPIO_PIN_RESET);

#endif

	lcd_address[0] = 0x00;
	lcd_address[1] = 0x40;
	lcd_address[2] = 0x00 + column;
	lcd_address[3] = 0x40 + column;

	lcd_max_column = column;
	lcd_max_row = row;

	lcd_rs_reset;


	HAL_Delay(50);
	lcd_4bit(0b0011);
	HAL_Delay(5);
	lcd_4bit(0b0011);
	HAL_Delay(1);
	lcd_4bit(0b0011);
	HAL_Delay(1);
	lcd_4bit(0b0010);
	HAL_Delay(1);
	lcd_8bit(0b00101000);
	lcd_8bit(0b00001000);
	HAL_Delay(1);
	lcd_8bit(0b00000001);
	HAL_Delay(3);
	lcd_8bit(0b00000110);
	HAL_Delay(1);
	lcd_8bit(0b00001100);
	HAL_Delay(1);

	//---------------------------------------
	//Tulisan angka berukuran besar
	//Segmen-segmennya dibuat sebanyak 8 buah
	//Semoga bermanfaat
	//---------------------------------------

	//Tepi kiri atas
	lcd_rs_reset;
	lcd_8bit(0x40);
	lcd_rs_set;
	lcd_8bit(0b00111);
	lcd_8bit(0b01111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);

	//Tepi kiri bawah
	lcd_rs_reset;
	lcd_8bit(0x48);
	lcd_rs_set;
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b01111);
	lcd_8bit(0b00111);

	//Tepi kanan atas
	lcd_rs_reset;
	lcd_8bit(0x50);
	lcd_rs_set;
	lcd_8bit(0b11100);
	lcd_8bit(0b11110);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);

	//Tepi kanan bawah
	lcd_rs_reset;
	lcd_8bit(0x58);
	lcd_rs_set;
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11110);
	lcd_8bit(0b11100);

	//Tengah-Atas
	lcd_rs_reset;
	lcd_8bit(0x60);
	lcd_rs_set;
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);

	//Tengah-Tengah
	lcd_rs_reset;
	lcd_8bit(0x68);
	lcd_rs_set;
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);

	//Tengah-Bawah
	lcd_rs_reset;
	lcd_8bit(0x70);
	lcd_rs_set;
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b00000);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);

	//Penuh
	lcd_rs_reset;
	lcd_8bit(0x78);
	lcd_rs_set;
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
	lcd_8bit(0b11111);
}

void lcd_gotoxy(uint8_t column, uint8_t row)
{
	if(column >= lcd_max_column) column = 0;
	if(row >= lcd_max_row) row = 0;

	lcd_rs_reset;
	lcd_8bit(0x80 | (lcd_address[row] + column));

	while(++i<wait){}
	i=0;

}

void lcd_clear(void)
{
	lcd_rs_reset;
	lcd_8bit(0x01);
	HAL_Delay(3);
}

void lcd_putc(uint8_t data)
{
	lcd_rs_set;
	lcd_8bit(data);

	while(++i<wait){}
	i=0;

}

void lcd_print(uint8_t column, uint8_t row, char* data)
{
	uint8_t current_column = column;
	uint8_t current_row = row;

	while(*data)
	{
		if(current_column >= lcd_max_column && current_row >= lcd_max_row)
		{
			current_column = 0;
			current_row = 0;

			lcd_gotoxy(current_column,current_row);
			lcd_putc(*data++);
			current_column++;
		}
		else if(current_column >= lcd_max_column)
		{
			current_column = 0;
			current_row++;

			lcd_gotoxy(current_column,current_row);
			lcd_putc(*data++);
			current_column++;
		}
		else
		{
			lcd_gotoxy(current_column,current_row);
			lcd_putc(*data++);
			current_column++;
		}
	}
}

void lcd_4bit(uint8_t data)
{

	HAL_GPIO_WritePin(lcd_db7_GPIO_Port,lcd_db7_Pin,(data & 0x08)>>3);
	HAL_GPIO_WritePin(lcd_db6_GPIO_Port,lcd_db6_Pin,(data & 0x04)>>2);
	HAL_GPIO_WritePin(lcd_db5_GPIO_Port,lcd_db5_Pin,(data & 0x02)>>1);
	HAL_GPIO_WritePin(lcd_db4_GPIO_Port,lcd_db4_Pin,(data & 0x01)>>0);




	while(++i<wait){}
	i=0;
	lcd_e_set;
	while(++i<wait){}
	i=0;

	lcd_e_reset;
	while(++i<wait){}
	i=0;
	lcd_e_set;
}

void lcd_8bit(uint8_t data)
{
	HAL_GPIO_WritePin(lcd_db7_GPIO_Port,lcd_db7_Pin,(data & 0x80)>>7);
	HAL_GPIO_WritePin(lcd_db6_GPIO_Port,lcd_db6_Pin,(data & 0x40)>>6);
	HAL_GPIO_WritePin(lcd_db5_GPIO_Port,lcd_db5_Pin,(data & 0x20)>>5);
	HAL_GPIO_WritePin(lcd_db4_GPIO_Port,lcd_db4_Pin,(data & 0x10)>>4);


  	while(++i<wait){}
	i=0;
	lcd_e_set;
	while(++i<wait){}
	i=0;
	lcd_e_reset;
	while(++i<wait){}
	i=0;
	lcd_e_set;


	HAL_GPIO_WritePin(lcd_db7_GPIO_Port,lcd_db7_Pin,(data & 0x08)>>3);
	HAL_GPIO_WritePin(lcd_db6_GPIO_Port,lcd_db6_Pin,(data & 0x04)>>2);
	HAL_GPIO_WritePin(lcd_db5_GPIO_Port,lcd_db5_Pin,(data & 0x02)>>1);
	HAL_GPIO_WritePin(lcd_db4_GPIO_Port,lcd_db4_Pin,(data & 0x01)>>0);

	while(++i<wait){}
	i=0;
	lcd_e_set;
	while(++i<wait){}
	i=0;
	lcd_e_reset;
	while(++i<wait){}
	i=0;
	lcd_e_set;
}




