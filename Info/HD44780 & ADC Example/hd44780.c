//------------------------------------------------------
/* File:       Library for HD44780 compatible displays  */
/* Version:	   v1.1  						 			*/
/* Language	   ANSI C			   		  	 			*/
/* Author:     GrAnd/www.MakeSystem.net		 			*/
/* Tested on:  AVR		  			 	 	 		 	*/
/* License:	   GNU LGPLv2.1		 		 	 			*/
//------------------------------------------------------
/* Copyright (C)2012 GrAnd. All right reserved 			*/
//------------------------------------------------------

/*
  [hd44780.h - Library for HD44780 compatible displays]
  [Copyright (C)2012 GrAnd. All right reserved]

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

Contact information :
						mail@makesystem.net
						http://makesystem.net/?page_id=2
*/

#include "hd44780.h"

#if ( USE_PROGRESS_BAR )
static int8u_t progress_bar[NUMBER_OF_CELL_ELEMENTS] = {0x00,0x10,0x18,0x1C,0x1E,0x1F};
static int8u_t current_bar_load;
#endif

//-------------------------------
// LOW LEVEL FUNCTIONS
//-------------------------------
static void DELAY(int16u_t ms);
static void LCD_STROBE(int16u_t loop);
static void HIGHBITS(int8u_t data);
static void LOWBITS(int8u_t data);

//-------------------------------
/* DELAY FUNCTION */
//-------------------------------
static void DELAY(volatile int16u_t ms)
{
 volatile int16u_t alfa,beta;
 for(alfa=0;alfa<ms;alfa++)
  for(beta=0;beta<MCU_WAIT_CYCLES;beta++)
  ;
}

//-------------------------------
/* INITIATE TRANSFER OF DATA/COMMAND TO LCD */
//-------------------------------
static void LCD_STROBE(int16u_t loop)
{
 ENABLE(LCD_WIRE,E);
 DELAY(MCU_CLK);
 DISABLE(LCD_WIRE,E); // Enter
 DELAY(loop);
}

//-------------------------------
/* PUT HIGH BITS */
//-------------------------------
static void HIGHBITS(int8u_t data)
{
 if(data & 0x80) ENABLE(LCD_WIRE,D7); else DISABLE(LCD_WIRE,D7);
 if(data & 0x40) ENABLE(LCD_WIRE,D6); else DISABLE(LCD_WIRE,D6);
 if(data & 0x20) ENABLE(LCD_WIRE,D5); else DISABLE(LCD_WIRE,D5);
 if(data & 0x10) ENABLE(LCD_WIRE,D4); else DISABLE(LCD_WIRE,D4);
}

//-------------------------------
/* PUT LOW BITS */
//-------------------------------
static void LOWBITS(int8u_t data)
{
 if(data & 0x08) ENABLE(LCD_WIRE,D7); else DISABLE(LCD_WIRE,D7);
 if(data & 0x04) ENABLE(LCD_WIRE,D6); else DISABLE(LCD_WIRE,D6);
 if(data & 0x02) ENABLE(LCD_WIRE,D5); else DISABLE(LCD_WIRE,D5);
 if(data & 0x01) ENABLE(LCD_WIRE,D4); else DISABLE(LCD_WIRE,D4);
}

//-------------------------------
/* PUT DATA/COMMAND TO LCD */
//-------------------------------
void lcd_cmd(int8u_t data, int16u_t loop)
{/* LCD ELEMENTARY COMMAND */
 HIGHBITS(data);
 LCD_STROBE(0);
 LOWBITS(data);
 LCD_STROBE(loop); // busy delay
}

 				   	  	   	   //-------------------------------
							   /*         LCDlib API          */
							   //-------------------------------

//-------------------------------
/* LCD CLEAR SCREEN */
//-------------------------------
void lcd_clrscr(void)
{
 lcd_cmd(0x01,2); // clear screen
}

//-------------------------------
/* LCD RETURN CURSOR */
//-------------------------------
void lcd_return(void)
{
 lcd_cmd(0x02,2); // return cursor
}

//-------------------------------
/* GO TO SPECIFIED MEMORY ADDRESS */
//-------------------------------
void lcd_goto(int8u_t line, int8u_t address)
{/* GO TO SPECIFIED ADDRESS */
 switch(line)
 {
  case     1: lcd_cmd(0x80|address,0); break;
  case     2: lcd_cmd(0xC0|address,0); break;
  case CGRAM: lcd_cmd(0x40|address,0); break; // CGRAM address
 }
}

//-------------------------------
/* WRITE ENTIRE STRING 
   TO SPECIFIED MEMORY */
//-------------------------------
void lcd_prints(const char *p)
{/* WRITE A STRING TO LCD */
 ENABLE(LCD_WIRE,RS);
  while(*p)
  {
#if ( USE_FORMATTED_OUTPUT )
//-------------------------------
// new line
//-------------------------------
   if((*p == '\n'))
   {
	DISABLE(LCD_WIRE,RS);
	lcd_goto(2,0);
	ENABLE(LCD_WIRE,RS);
	p++;
   }
//-------------------------------
// return
//-------------------------------
   else if((*p == '\r'))
   {
	DISABLE(LCD_WIRE,RS);
	lcd_return();
	ENABLE(LCD_WIRE,RS);
	p++;
   }
//-------------------------------
// tab
//-------------------------------
   else if((*p == '\t'))
   {
	DISABLE(LCD_WIRE,RS);
	switch(TAB_SPACE)
    {
	 case 8: lcd_cmd(0x14,0); // cursor right shift
     	  	 lcd_cmd(0x14,0); // cursor right shift
     		 lcd_cmd(0x14,0); // cursor right shift
     		 lcd_cmd(0x14,0); // cursor right shift
     case 4: lcd_cmd(0x14,0); // cursor right shift
     	  	 lcd_cmd(0x14,0); // cursor right shift
     case 2: lcd_cmd(0x14,0); // cursor right shift
     case 1: lcd_cmd(0x14,0); // cursor right shift
    }
	ENABLE(LCD_WIRE,RS);
	p++;
   }
//-------------------------------
// display
//-------------------------------
   else
#endif
    lcd_cmd(*p++,0);
 }
 DISABLE(LCD_WIRE,RS);
}

//-------------------------------
/* WRITE A SINGLE CHARACTER 
   TO SPECIFIED MEMORY */
//-------------------------------
void lcd_putc(int8u_t data)
{/* WRITE A CHARACTER TO LCD */
 ENABLE(LCD_WIRE,RS);
 lcd_cmd(data,0);
 DISABLE(LCD_WIRE,RS);
}

//-------------------------------
/* LOAD USER-DEFINED CHARACTER IN CGRAM */
//-------------------------------
void lcd_load(int8u_t* vector, int8u_t position)
{/* USE CGRAM CHAR SPACE: 0 to 7 */
 int8u_t i;
 lcd_goto(CGRAM,position*DRAW_CHAR_SIZE);
 for(i=0;i<DRAW_CHAR_SIZE;i++)
  lcd_putc(vector[i]);
}

//-------------------------------
/* DISPLAY USER-DEFINED CHARACTER ON DDRAM */
//-------------------------------
void lcd_drawchar( int8u_t* vector, 
	 			   int8u_t position, 
	 			   int8u_t line, 
				   int8u_t address )
{/* USE CGRAM CHAR SPACE */
 lcd_load(vector,position);
 lcd_goto(line,address);
 lcd_putc(position);
}

//-------------------------------
/* ERASE A SINGLE CHARACTER 
   FROM DISPLAY */
//-------------------------------
void lcd_backspace(void)
{/* ERASE LEFT CHAR */
 lcd_cmd(0x10,0); // �������� ������ �� ���� ������� �����
 lcd_putc(' '); // �������, ����� ���� ���������� ������������� ������
 lcd_cmd(0x10,0); // �������� ������ �� ���� ������� �����
}

//-------------------------------
/* SCROLL DISPLAY 
   TO SPECIFIED DIRECTION */
//-------------------------------
void lcd_scroll(int8u_t direction)
{
 switch(direction)
 {
  case RIGHT : lcd_cmd(0x1C,0); break; // scroll display to right
  case LEFT  : lcd_cmd(0x18,0); break; // scroll display to left
 }
}

//-------------------------------
/* SHIFT CURSOR 
   TO SPECIFIED DIRECTION */
//-------------------------------
void cursor_shift(int8u_t direction)
{
 switch(direction)
 {
  case RIGHT : lcd_cmd(0x14,0); break; // shift cursor to right
  case LEFT  : lcd_cmd(0x10,0); break; // shift cursor to left
 }
}

//-------------------------------
/* DISPLAY A INTEGER NUMER */
//-------------------------------
void lcd_itostr(int32s_t value)
{/* DISPLAY A INTEGER NUMER: +/- 2147483647 */
 int32s_t i;
 if(value<0)
 {
  lcd_putc('-');
  value=-value;
 }
 for(i=1;(value/i)>9;i*=10);
 lcd_putc(value/i+'0');
 i/=10;
 while(i)
 {
  lcd_putc((value%(i*10))/i+'0');
  i/=10;
 }
}

//-------------------------------
/* DISPLAY A 4-DIGIT INTEGER NUMER */
//-------------------------------
void lcd_numTOstr(int16u_t value, int8u_t nDigit)
{/* DISPLAY n-DIGIT INTEGER NUMER */
 switch(nDigit)
 {
  case 4: lcd_putc((value/1000)+'0');
  case 3: lcd_putc(((value/100)%10)+'0');
  case 2: lcd_putc(((value/10)%10)+'0');
  case 1: lcd_putc((value%10)+'0');
 }
}

#if ( USE_PROGRESS_BAR )
//-------------------------------
/* PRELOAD PROGRESS BAR ELEMENTS IN CGRAM */
//-------------------------------
void lcd_readybar(void)
{
 int8u_t i,j;
 for(i=0;i<NUMBER_OF_CELL_ELEMENTS;i++)
 {
  lcd_goto(CGRAM,(i*DRAW_CHAR_SIZE));
  for(j=0;j<PROGRESS_BAR_HEIGHT;j++)
   lcd_putc(progress_bar[i]);
 }
 lcd_goto(1,0);
}

//-------------------------------
/* DRAW PROGRESS BAR ON DDRAM */
//-------------------------------
void lcd_drawbar(int8u_t next_bar_load)
{
 int8u_t i = current_bar_load;
 int8u_t cell = (current_bar_load/FULL_ELEMENT); // find current cell position in progress bar
 if(next_bar_load > NUMBER_OF_BAR_ELEMENTS ) next_bar_load = NUMBER_OF_BAR_ELEMENTS;
 if( next_bar_load > current_bar_load )
 {
//-------------------------------
// increment progress bar code //
//-------------------------------
  lcd_goto(DRAW_PROGRESS_BAR_ON_LINE, cell); // goto current cell position
  while( i != next_bar_load )
  {
   i++;
   if( CELL_RATIO(i) == 0 ) lcd_putc( FULL_ELEMENT );
    else lcd_putc( CELL_RATIO(i) );
   if( CELL_RATIO(i) ) cursor_shift(LEFT);
  }
 }
 else
 {
//-------------------------------
// decrement progress bar code //
//-------------------------------
  if(CELL_RATIO(i) == 0) cell--;
  lcd_goto(DRAW_PROGRESS_BAR_ON_LINE, cell); // goto current cell position
  lcd_cmd(ENTRY_MODE_DEC_NO_SHIFT,0); // decrement lcd cursor
  while( i != next_bar_load )
  {
   i--;
   lcd_putc( CELL_RATIO(i) );
   if( CELL_RATIO(i) ) cursor_shift(RIGHT);
  }
  lcd_cmd(ENTRY_MODE_INC_NO_SHIFT,0); // increment lcd cursor
 }
//-------------------------------
//       store new value       //
//-------------------------------
 current_bar_load = next_bar_load;
}

//-------------------------------
/*  CLEAR ENTYRE PROGRESS BAR  */
//-------------------------------
void lcd_clearbar(void)
{
 int8u_t i;
 lcd_goto(DRAW_PROGRESS_BAR_ON_LINE, (PROGRESS_BAR_WIDTH - 1));
 lcd_cmd(ENTRY_MODE_DEC_NO_SHIFT,0); // decrement
 for(i=0;i<PROGRESS_BAR_WIDTH;i++)
  lcd_putc(' ');
 lcd_cmd(ENTRY_MODE_INC_NO_SHIFT,0); // increment
 current_bar_load = 0;
}
#endif

//-------------------------------
/* CONFIGURE 4-BIT DISPLAY INTERFACE */
//-------------------------------
void lcd_config(int8u_t param)
{/* CONFIGURE THE DISPLAY */
 HIGHBITS(param); // 4-bit, two lines, 5x8 pixel
  LCD_STROBE(0); // change 8-bit interface to 4-bit interface
  LCD_STROBE(0); // init 4-bit interface
 LOWBITS(param);
  LCD_STROBE(0);
}

//-------------------------------
/* INITIALIZE ENTIRE DISPLAY */
//-------------------------------
void lcd_init(void)
{
 lcd_config(DEFAULT_DISPLAY_CONFIG); // 1, Data Lenght, Number of lines, character font
 lcd_cmd(DEFAULT_DISPLAY_CONTROL,0); // 1, lcd, cursor, blink
 lcd_cmd(DEFAULT_ENTRY_MODE,0); // 1,increment/decrement,display shift on/off 
 lcd_cmd(0x01,2); // clear display
 lcd_cmd(0x02,2); // 1, return home cursor
#if (USE_PROGRESS_BAR)
 lcd_readybar();
#endif
}

//-------------------------------
/* END OF FILE */
//-------------------------------
