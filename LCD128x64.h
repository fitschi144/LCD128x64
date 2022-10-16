/*
  Library:        LC128x64 - Display Controll
  Written by:     Fiete Fellenberg
  Matrikel:  	  XXXXXXXX
  Date Written:   01/05/2021
  Updated:        23/05/2021
  Description:    this script controlls the basic functions of the 128x64 display.
 */

#ifndef LCD128X64_H_
#define LCD128X64_H_

#include <stdbool.h>
#include "main.h"

void lcd128_init_8bits(
    GPIO_TypeDef* port_rs_e, uint16_t rs_pin, uint16_t e_pin,
    GPIO_TypeDef* port_0_3, uint16_t d0_pin, uint16_t d1_pin, uint16_t d2_pin, uint16_t d3_pin,
    GPIO_TypeDef* port_4_7, uint16_t d4_pin, uint16_t d5_pin, uint16_t d6_pin, uint16_t d7_pin,
	GPIO_TypeDef* port_CS1_RSTB, uint16_t CS1_pin, uint16_t CS2_pin, uint16_t RSTB_pin);


#endif /* LCD128X64_H_ */
/**
 * @brief Print to display any datatype (e.g. lcd128x64_printf(x,y,"Value1 = %.1f", 123.45))
 */
void lcd128_printf(int x,int y,const char* str, ...);

void LCD128_ClearAll();

void LCD128_BlackOut();

void Center_Text(int y,const char* str, ...);

void Allign_Text_Right (int x, int y, const char* str, ...);

void Allign_Text_Left (int x, int y, const char* str, ...);

void LCD128_logo(int hor);

void LCD128_Figure(int number, int x, int y);

int LCD128_obstical(int number, int x, int y, int x_fig, int y_fig, int refr);

void LCD128_Jump(int x, int y, int z);

void LCD128_Clear_selected(int x, int y);

void LCD_Graph (int x, int y, int i, int j);
