/*
 * TP-Meter.c
 *
 * Created:     21.08.2015 16:36:53
 * CPU:         ATMega48V
 * LCD:         WH1602
 * Frequency:   8 MHz
 * Author:      Symrak
 *
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include "Libs\hd44780.h"

uint8_t mode = 0;

void startup()
{
	/* ������������� ������ */
    DDRB  = 0xFF;				// ���� B �� �����
	PORTB = 0x00;				// ���. 0 �� �����
    
	/* ������� ��� �������� � ������� */
    uint8_t degree[8]=
	{
    	0b00001110,
    	0b00001010,
    	0b00001110,
    	0b00000000,
    	0b00000000,
    	0b00000000,
    	0b00000000,
    	0b00000000
	};      
    
    /* ������������� ������� */
    lcd_init();				// ������������� �������
    lcd_clrscr();           // ������� �������
	lcd_load(degree,0);		// �������� ������� ������� �� �������
    lcd_goto(1,0);			// ������� �� ������ 0 ������ 1
}
void calibrating()
{
    /* ��������� ���������� ��� ��� ���������� ����� */
    lcd_clrscr();
    lcd_goto(1,0);
    lcd_prints("\tTo calibrate");
    lcd_goto(2,0);
    lcd_prints("\t Press MODE");
    if (mode == 0)
    {
        lcd_clrscr();
        lcd_goto(1,0);
        lcd_prints(" Calibrating...");
        lcd_goto(2,0);
        for(int i=0;i<NUMBER_OF_BAR_ELEMENTS;i++)
        {
            lcd_drawbar(i);
            _delay_ms(5);
        }
    }
}

int main(void)
{
    startup();
    calibrating();
    while(1)
    {
        //TODO:: Please write your application code 
    }
    return 0;
}