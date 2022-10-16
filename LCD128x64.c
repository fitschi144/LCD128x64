/*
  Library:        LC128x64 - Display Control
  Written by:     Fiete Fellenberg
  Matrikel:  	  XXXXXXXX
  Date Written:   01/05/2021
  Updated:        01/06/2022
  Description:    this script controls the basic functions of the 128x64 display.

  References**:

  (1) STM32L476xx Datasheet
  	  Link: https://www.st.com/resource/en/datasheet/stm32l476je.pdf
  	  Date: June 2019
  (2) DEM128064 Datasheet
  	  Link: https://www.display-elektronik.de/filter/DEM128064B_SYH.pdf
  	  Date: Oct 2006
 */

#include "LCD128x64.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "Font_Header.h"

/* LCD Library Variables */
static GPIO_TypeDef* PORT_RS_and_E;               // RS and E PORT
static uint16_t PIN_RS, PIN_E;                    // RS and E pins
static GPIO_TypeDef* PORT_CS1_RSTB;               // CS and RSTB PORT
static uint16_t CS1_PIN, CS2_PIN, RSTB_PIN;       // CS and RSTB pins
static GPIO_TypeDef* PORT_LSB;                    // LSBs D0, D1, D2 and D3 PORT
static uint16_t D0_PIN, D1_PIN, D2_PIN, D3_PIN;   // LSBs D0, D1, D2 and D3 pins
static GPIO_TypeDef* PORT_MSB;                    // MSBs D5, D6, D7 and D8 PORT
static uint16_t D4_PIN, D5_PIN, D6_PIN, D7_PIN;   // MSBs D5, D6, D7 and D8 pins
#define T_CONST   50							  	  // Time Constant

#define lcd_CHIP0 HAL_GPIO_WritePin(PORT_CS1_RSTB, CS2_PIN, GPIO_PIN_RESET),   HAL_GPIO_WritePin(PORT_CS1_RSTB, CS1_PIN, GPIO_PIN_SET)
#define lcd_CHIP1 HAL_GPIO_WritePin(PORT_CS1_RSTB, CS2_PIN, GPIO_PIN_SET),   HAL_GPIO_WritePin(PORT_CS1_RSTB, CS1_PIN, GPIO_PIN_RESET)
#define lcd_ALL HAL_GPIO_WritePin(PORT_CS1_RSTB, CS2_PIN, GPIO_PIN_SET),   HAL_GPIO_WritePin(PORT_CS1_RSTB, CS1_PIN, GPIO_PIN_SET)

/*
 * LCD commands -------------------------------------------------------------
 */

#define LCD_ON								0x3F
#define LCD_OFF								0x3E
#define LCD_SET_ADD_X						0x40
#define LCD_DISP_START_Line					0xC0
#define LCD_SET_ADD_Y						0xB8


/* private functions prototypes */
/**
 * @brief DWT Cortex Tick counter for Microsecond delay
 */
static uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  /* Enable TRC */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  /* Enable clock cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;
  /* 3 NO OPERATION instructions */
  __NOP();
  __NOP();
  __NOP();
  /* Check if clock cycle counter has started */
  if(DWT->CYCCNT)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t usec)
{
 uint32_t clk_cycle_start = DWT->CYCCNT;
 usec *= (HAL_RCC_GetHCLKFreq() / 1000000);
 while ((DWT->CYCCNT - clk_cycle_start) < usec);
}

/**
 * @brief RS control
 */
static void lcd128_rs(bool state)
{
  HAL_GPIO_WritePin(PORT_RS_and_E, PIN_RS, (GPIO_PinState)state);
}

/**
 * @brief Enable Pulse function
 */
static void lcd128_enablePulse(void)
{
  DWT_Delay_us(T_CONST);
  HAL_GPIO_WritePin(PORT_RS_and_E, PIN_E, GPIO_PIN_SET);
  DWT_Delay_us(T_CONST);
  HAL_GPIO_WritePin(PORT_RS_and_E, PIN_E, GPIO_PIN_RESET);
  DWT_Delay_us(T_CONST);
}

/**
 * @brief Write parallel signal to lcd
 */
static void lcd128_write(uint8_t wbyte)
{
	uint8_t LSB_nibble = wbyte&0xF, MSB_nibble = (wbyte>>4)&0xF;
	//LSB data
	HAL_GPIO_WritePin(PORT_LSB, D0_PIN, (GPIO_PinState)(LSB_nibble&0x1));
	HAL_GPIO_WritePin(PORT_LSB, D1_PIN, (GPIO_PinState)(LSB_nibble&0x2));
	HAL_GPIO_WritePin(PORT_LSB, D2_PIN, (GPIO_PinState)(LSB_nibble&0x4));
	HAL_GPIO_WritePin(PORT_LSB, D3_PIN, (GPIO_PinState)(LSB_nibble&0x8));
	//MSB data
	HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(MSB_nibble&0x1));
	HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(MSB_nibble&0x2));
	HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(MSB_nibble&0x4));
	HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(MSB_nibble&0x8));
	lcd128_enablePulse();
}

/**
 * @brief Write command
 */
static void lcd128_writeCommand(uint8_t cmd)
{
	lcd128_rs(false);
	lcd128_write(cmd);
	lcd128_rs(true);
}
/**
 * @brief Write data
 */
static void lcd128_writeData(uint8_t data)
{
  lcd128_rs(true);
  lcd128_write(data);
}
/**
 * @brief Initialize LCD on 8-bits mode
 * @param[in] *port_rs_e RS and EN GPIO Port (e.g. GPIOB)
 * @param[in] *port_0_3 D0 to D3 GPIO Port
 * @param[in] *port_4_7 D4 to D7 GPIO Port
 * @param[in] * CS1 CS2 RSTB GPIO Port
 * @param[in] x_pin GPIO pin (e.g. GPIO_PIN_1)
 */
void lcd128_init_8bits(
    GPIO_TypeDef* port_rs_e, uint16_t rs_pin, uint16_t e_pin,
    GPIO_TypeDef* port_0_3, uint16_t d0_pin, uint16_t d1_pin, uint16_t d2_pin, uint16_t d3_pin,
    GPIO_TypeDef* port_4_7, uint16_t d4_pin, uint16_t d5_pin, uint16_t d6_pin, uint16_t d7_pin,
	GPIO_TypeDef* port_CS1_RSTB, uint16_t CS1_pin, uint16_t CS2_pin, uint16_t RSTB_pin)
{
  DWT_Delay_Init();
  //Set GPIO Ports and Pins data
  PORT_RS_and_E = port_rs_e;
  PIN_RS = rs_pin;
  PIN_E = e_pin;

  PORT_CS1_RSTB = port_CS1_RSTB;
  CS1_PIN = CS1_pin;
  CS2_PIN = CS2_pin;
  RSTB_PIN = RSTB_pin;

  PORT_LSB = port_0_3;
  D0_PIN = d0_pin;
  D1_PIN = d1_pin;
  D2_PIN = d2_pin;
  D3_PIN = d3_pin;

  PORT_MSB = port_4_7;
  D4_PIN = d4_pin;
  D5_PIN = d5_pin;
  D6_PIN = d6_pin;
  D7_PIN = d7_pin;

  //Initialize LCD
  //Set All Command Pins to High
  HAL_GPIO_WritePin(PORT_CS1_RSTB, RSTB_PIN, GPIO_PIN_RESET);
  //1. Wait at least 15ms
  HAL_Delay(5);
  lcd_ALL;
  HAL_GPIO_WritePin(PORT_CS1_RSTB, RSTB_PIN, GPIO_PIN_SET);
  //2. Attentions sequence
  lcd128_writeCommand(LCD_SET_ADD_X); // set X Address to 0
  lcd128_writeCommand(LCD_SET_ADD_Y); // set Y Address to 0
  lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
  lcd128_writeCommand(LCD_ON); // Turn the LCD On
  LCD128_ClearAll();

}
/**
 * @brief Celars all Pixel on the Display
 */
void LCD128_ClearAll()
{
	int i,j;
	//Select both left & right half of display
	lcd_ALL;
	//Write zeros to each Column
	for(i = LCD_SET_ADD_Y; i < 0xC0; i++)
	{
		lcd128_writeCommand(i);
		for(j = 0; j < 64; j++)
		{
			lcd128_writeData(0x00);
		}
	}
}

/**
 * @brief Blacks the Display Out
 */
void LCD128_BlackOut()
{
	int i,j;
	//Select both left & right half of display
	lcd_ALL;
	//Write ones to each Column
	for(i = LCD_SET_ADD_Y; i < 0xC0; i++)
	{
		lcd128_writeCommand(i);
		for(j = 0; j < 64; j++)
		{
			lcd128_writeData(0xFF);
		}
	}
}


/**
 * @brief Prints a Nice Headline (must be integrated before)
 */
void LCD128_logo(int hor)
{
	int i,j,k;
	lcd_ALL;
	k=0;
	lcd128_writeCommand(LCD_SET_ADD_X); // set X Address to 0
	for(i = LCD_SET_ADD_Y; i < 0xC0; i++){
		for(j = 0; j < 128; j++){
			if(j>63){
				lcd_CHIP1;
				lcd128_writeCommand(i);
				lcd128_writeData(logo[hor][j+k*128]);
			}
			else{
				lcd_CHIP0;
				lcd128_writeCommand(i);
				lcd128_writeData(logo[hor][j+k*128]);
			}
		}
		k=k+1;
	}
}

/**
 * @brief Prints a Game Figure (must be integrated before)
 */
void LCD128_Figure(int number, int x, int y)
{
	if(x<0){
			return;
		}
		else{
			int i,j;
			lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
			for(i = 0; i < 2; i++){
				for(j = 0; j < 30; j++){
					if(j+x>63){
						lcd_CHIP1;
						lcd128_writeCommand(LCD_SET_ADD_Y+y+i);
						lcd128_writeCommand(LCD_SET_ADD_X+x-64+j); // set x address with offset
						lcd128_writeData(Game[number][j+i*30]);
					}
					else{
						lcd_CHIP0;
						lcd128_writeCommand(LCD_SET_ADD_Y+y+i);
						lcd128_writeCommand(LCD_SET_ADD_X+x+j); // set x address with offset
						lcd128_writeData(Game[number][j+i*30]);
					}
				}
			}
		}
	//HAL_Delay(10);
}
/**
 * @brief Prints a Game Figure (must be integrated before)
 */
int LCD128_obstical(int number, int x, int y, int x_fig, int y_fig, int refr)
{
	int j;
	x=128-x;
	if (refr==1){
		LCD128_Clear_selected(x_fig,y_fig);
	}
	lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
	for(j = 0; j < 14; j++){
		if(j+x>63){
			if(j+x<128){
				lcd_CHIP1;
				lcd128_writeCommand(LCD_SET_ADD_Y+y+1);
				lcd128_writeCommand(LCD_SET_ADD_X+x-64+j); // set x address with offset
				lcd128_writeData(obsticle[number][j]);
			}
		}
		else{
			if(j+x>=0){
				lcd_CHIP0;
				lcd128_writeCommand(LCD_SET_ADD_Y+y+1);
				lcd128_writeCommand(LCD_SET_ADD_X+x+j); // set x address with offset
				lcd128_writeData(obsticle[number][j]);
			}
		}
	}
	if(y_fig==y){//checking for crash
		if(x<x_fig+30){
				if(x+14>x_fig){
					return x-x_fig-30;
				}
			}
	}
	return 0;

}
/**
 * @brief Prints a jumping Figure (must be integrated before)
 * @param[in] *X offset from the Left side
 * @param[in] *Y offset from the upper side
 * @param[in] *Z 0=up 1 down
 */
void LCD128_Jump(int x, int y, int z)
{
	int i,j,k,p;
	lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
	for(k = 0; k < 6; k++){
		if(z==0){
			p=5-k;
		}else{
			p=k;
		}
		for(i = 0; i < 3; i++){
			for(j = 0; j < 30; j++){
				if(j+x>63){
					lcd_CHIP1;
					lcd128_writeCommand(LCD_SET_ADD_Y+y+i);
					lcd128_writeCommand(LCD_SET_ADD_X+x-64+j); // set x address with offset
					lcd128_writeData(jump[p][j+i*30]);
				}
				else{
					lcd_CHIP0;
					lcd128_writeCommand(LCD_SET_ADD_Y+y+i);
					lcd128_writeCommand(LCD_SET_ADD_X+x+j); // set x address with offset
					lcd128_writeData(jump[p][j+i*30]);
				}
			}
		}
	}
	LCD128_Clear_selected(x,y);
}
/**
 * @brief Clears the Selected section of the display
 * @param[in] *X offset from the Left side
 * @param[in] *Y offset from the upper side
 */
void LCD128_Clear_selected(int x, int y)
{
	int i,j;
	lcd_ALL;
	//Write zeros to each Column
	for(i = LCD_SET_ADD_Y+y; i < 0xC0; i++)
	{
		lcd128_writeCommand(i);
		for(j = 0; j < 64; j++)
		{
			lcd128_writeData(0x00);
		}
	}
}
/**
 * @brief printf function for the Display, linewhise (e.g. lcd128x64_printf(5,7,"Value1 = %.1f", 123.45))
 * @param[in] *X offset from the right side
 * @param[in] *Y offset from the upper side
 * @param[in] *Value to print with datatype (e.g. %f -float or nothing = char)
 */
void lcd128_printf(int x,int y,const char* str, ...)
{
	  //Define input Variables
	  uint8_t dat;
	  uint8_t k=0;
	  char stringArray[25];
	  //Checking for Ascii numbers
	  va_list args;
	  va_start(args, str);
	  vsprintf(stringArray, str, args);
	  va_end(args);
	  //Set Position
	  if (x+strlen(stringArray)*FONT_SIZE < 128 && y < 8){
		  if (x+strlen(stringArray)*FONT_SIZE < 64){
			  lcd_CHIP0; //Select Chip 0
			  lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
			  lcd128_writeCommand(LCD_SET_ADD_X+x); // set x address with offset
			  lcd128_writeCommand(LCD_SET_ADD_Y+y); // set y address with offset
			  for(uint8_t i=0;  i<strlen(stringArray); i++){
				  for(uint8_t j=0;  j<FONT_SIZE; j++){
					  dat= A_GlcdFontTable_U8[(uint8_t)stringArray[i]-0x20][j]; //get ASCII Symbol, but with offset from the Table (-0x20)
					  lcd128_write(dat); //Print each column
				  }
			 }
		  }
		  else if(x+strlen(stringArray)*FONT_SIZE > 64 && x-64 > 0){
			  lcd_CHIP1; //Select Chip 1
			  lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
			  lcd128_writeCommand(LCD_SET_ADD_X+x-64); // set x address with offset
			  lcd128_writeCommand(LCD_SET_ADD_Y+y); // set y address with offset
			  for(uint8_t i=0;  i<strlen(stringArray); i++){
				  for(uint8_t j=0;  j<FONT_SIZE; j++){
					  dat= A_GlcdFontTable_U8[(uint8_t)stringArray[i]-0x20][j]; //get ASCII Symbol, but with offset from the Table (-0x20)
					  lcd128_write(dat); //Print each colum
				  }
			 }
		  }
		  else{
			  	  lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
			  	  lcd128_writeCommand(LCD_SET_ADD_Y+y); // set y address with offset
			  	  lcd128_writeCommand(LCD_SET_ADD_X+x); // set x address with offset
			  	  for(uint8_t i=0;  i<strlen(stringArray); i++){
			  		  if ((i+1)*FONT_SIZE+x<64){
			  			  lcd_CHIP0; //Select Chip 0
			  			  lcd128_writeCommand(LCD_SET_ADD_Y+y); // set y address with offset
			  			  lcd128_writeCommand(LCD_SET_ADD_X+x+((i)*FONT_SIZE)); // set x address with offset
						  for(uint8_t j=0;  j<FONT_SIZE; j++){
							  dat= A_GlcdFontTable_U8[(uint8_t)stringArray[i]-0x20][j]; //get ASCII Symbol, but with offset from the Table (-0x20)
							  lcd128_write(dat); //Print each column
						  }
			  		  }
			  		  else if((i+1)*FONT_SIZE+x>63 && (i+1)*FONT_SIZE+x<64+FONT_SIZE){
			  			  	lcd_CHIP0;
			  			  	lcd128_writeCommand(LCD_SET_ADD_X+x+((i)*FONT_SIZE)); // set x address with offset
							for(uint8_t j=0;  j<FONT_SIZE; j++){
								dat= A_GlcdFontTable_U8[(uint8_t)stringArray[i]-0x20][j];
								if (j+i*FONT_SIZE+x < 64){
									lcd128_write(dat); //Print each column
								}
								else{
									lcd_CHIP1;
									lcd128_writeCommand(LCD_SET_ADD_Y+y); // set y address with offset
									lcd128_writeCommand(LCD_SET_ADD_X+(j+i*FONT_SIZE+x-64));
									lcd128_write(dat);
								}
							}
			  		  }
			  		  else{
			  			  lcd_CHIP1;
			  			  lcd128_writeCommand(LCD_SET_ADD_X+(i*FONT_SIZE+x-64));
			  			  	  for(uint8_t j=0;  j<FONT_SIZE; j++){
								  dat= A_GlcdFontTable_U8[(uint8_t)stringArray[i]-0x20][j]; //get ASCII Symbol, but with offset from the Table (-0x20)
								  lcd128_write(dat); //Print each column
							  }
			  		  }
			  	  }
		  }
	  }
	  else{
		  lcd128_printf(14,3,"ERROR, TEXT ZU LANG!");
	  }
}

/**
 * @brief Centers Text on 128x64
 * @param[in] *text to center (max 25. characters)
 * @param[in] *y row from top)
 */
void Center_Text(int y, const char* str, ...){
	char stringArray[MAX_STRING_SIZE];
	va_list args;
	va_start(args, str);
	vsprintf(stringArray, str, args);
	va_end(args);
	int laenge = strlen(stringArray);
	laenge = 128 - laenge*FONT_SIZE;
	laenge /= 2;
	lcd128_printf(laenge, y, "%s", stringArray);
}

/**
 * @brief Offsets Text on 128x64 from the right side
 * @param[in] *text (max 25 characters)
 * @param[in] *x Offset in px
 * @param[in] *y row from top
 */
void Allign_Text_Right (int x, int y, const char* str, ...){
	char stringArray[MAX_STRING_SIZE];
	va_list args;
	va_start(args, str);
	vsprintf(stringArray, str, args);
	va_end(args);
	int laenge = strlen(stringArray);
	laenge = 128 - laenge*FONT_SIZE;
	laenge -= x;
	lcd128_printf(laenge, y, "%s", stringArray);
}

/**
 * @brief Offsets Text on 128x64 from the left side
 * @param[in] *text (max 25 characters)
 * @param[in] *x Offset in px
 * @param[in] *y row from top
 */
void Allign_Text_Left (int x, int y, const char* str, ...){
	char stringArray[MAX_STRING_SIZE];
	va_list args;
	va_start(args, str);
	vsprintf(stringArray, str, args);
	va_end(args);
	lcd128_printf(x, y, "%s", stringArray);
}

/**
 * @brief Draws Graph
 * @param[in] *y column
 * @param[in] *x x-achses position
 * @param[in] *n x offset in px
 * @param[in] *k y offset in
 */
void LCD_Graph (int x, int y, int n, int k){
	lcd128_writeCommand(LCD_SET_ADD_X); // set X Address to 0
	lcd128_writeCommand(LCD_SET_ADD_Y); // set Y Address to 0
	lcd128_writeCommand(LCD_DISP_START_Line); // set Start Line Register to 0
	//k=pow( 2, k ); -> delete pow function in every call
	if((x+n)>63){
		lcd_CHIP1;
		lcd128_writeCommand(0xBF-y);
		lcd128_writeCommand(LCD_SET_ADD_X+(x+n-64));
		lcd128_writeData(0x00+k);
	}
	else{
		lcd_CHIP0;
		lcd128_writeCommand(0xBF-y);
		lcd128_writeCommand(LCD_SET_ADD_X+x+n);
		lcd128_writeData(0x00+k);
	}
}
