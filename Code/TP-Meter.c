/*
 * TP-Meter.c
 *
 * Created:     21.08.2015 16:36:53
 * CPU:         ATMega8
 * LCD:         WH1602
 * Frequency:   8 MHz
 * Author:      Symrak
 *
 */ 

/* ����������� ������� ������� �� */
#define F_CPU           8000000

/* ����������� ��������� */
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "hd44780.h"

/* ����������� ����. �������� */
#define True            1
#define False           0

/* ����������� ������ */
#define MODE_BUTTON     PD6
#define PLUS_BUTTON     PD7
#define MINUS_BUTTON    PB0

/* ����������� ������� �������� (��) ��� ���������� �������� ��������� */
#define DEB_INT		    500

/* ����������� ������� ������ � ��������� */
/* ������ ������ */
#define WRK_MODE        1
#define SET_MODE        2
#define CAL_MODE        3
/* ���������� ������� */
#define MODE_CNT	    3
/* ��������� */
#define SET_TMP         0
#define SET_PWR         1
#define SET_VTG         2
#define SET_TIM         3
#define SET_BUZ	        4
#define SET_LUM		    5
#define SET_CTR	        6
/* ����� ���������� �������� � ���������� ��������� �������� */
#define SET_EASTER_EGG	True
#define SET_CNT_MAX		7
uint8_t SET_CNT       = 3;

/* ����������� ������ � ���� ��� ������������� �������� */
/* ������� � ��� ������������ ����������� */
#define temp_min        20
#define temp_max        90
#define temp_delta      10
/* ������� � ��� ������������ ������������ �������� */
#define power_min       100
#define power_max       9900
#define power_delta     100
/* ������� � ��� �������� �������� �������� ���������� */
#define voltage_min     180
#define voltage_max     250
#define voltage_delta   10
/* ������� � ��� ������� ���������� �������� */
#define timer_min       0
#define timer_max       120 
#define timer_delta     10
/* ������� � ��� �������� ������� ������� */
#define lcd_light_min   0
#define lcd_light_max   255
#define lcd_light_delta 5
/* ������� � ��� �������� ������������� ������� */
#define lcd_contr_min   0
#define lcd_contr_max   255
#define lcd_contr_delta 5
/* ���������� ���������� ���������� */
/* ���������� ������� ������ */
uint8_t  mode         = 0;
uint8_t  option       = 0;
uint8_t  launch       = True;
/* ����������, �������� �������� �������� */
uint8_t           set_max_tmp  = 0;
uint8_t  EEMEM ee_set_max_tmp  = 0;
uint16_t          set_max_pwr  = 0;
uint16_t EEMEM ee_set_max_pwr  = 0;
uint8_t           set_vtg      = 0;
uint8_t EEMEM  ee_set_vtg      = 0;
uint8_t           set_timer    = 0;
uint8_t EEMEM  ee_set_timer    = 0;
uint8_t           set_light    = 0;
uint8_t EEMEM  ee_set_light    = 0;
uint8_t           set_contrast = 0;
uint8_t EEMEM  ee_set_contrast = 0;
uint8_t           set_buzzer   = True;
uint8_t EEMEM  ee_set_buzzer   = True;
/* ������ ���������� � ����������� */
uint8_t timer_value   = 0;
#define CONTRAST OCR1B

void eeprom_load()
{
	/* ��������� ��� �������� �������� �� EEPROM */
	set_max_tmp = eeprom_read_byte(&ee_set_max_tmp);
	if((set_max_tmp < temp_min) || (set_max_tmp > temp_max))
		set_max_tmp = temp_min;
	
	set_max_pwr = eeprom_read_word(&ee_set_max_pwr);
	if((set_max_pwr < power_min) || (set_max_pwr > power_max))
		set_max_pwr = power_min;
	
	set_vtg = eeprom_read_byte(&ee_set_vtg);
	if((set_vtg < voltage_min) || (set_vtg > voltage_max))
		set_vtg = 220;
	
	set_timer = eeprom_read_byte(&ee_set_timer);
	if((set_timer < timer_min) || (set_timer > timer_max))
		set_timer = timer_min;

	set_light = eeprom_read_byte(&ee_set_light);
		if((set_light < lcd_light_min) || (set_light > lcd_light_max))
			set_light = lcd_light_max;

	set_contrast = eeprom_read_byte(&ee_set_contrast);
	if((set_contrast < lcd_contr_min) || (set_contrast > lcd_contr_max))
		set_contrast = lcd_contr_max;
	
	set_buzzer = eeprom_read_byte(&ee_set_buzzer);
	
}
void eeprom_save()
{
	/* ��������� ��� ���������� �������� � EEPROM */
	eeprom_write_byte(&ee_set_max_tmp, set_max_tmp);
	eeprom_write_word(&ee_set_max_pwr, set_max_pwr);
	eeprom_write_byte(&ee_set_vtg, set_vtg);
	eeprom_write_byte(&ee_set_timer, set_timer);
	eeprom_write_byte(&ee_set_light, set_light);
	eeprom_write_byte(&ee_set_contrast, set_contrast);
	eeprom_write_byte(&ee_set_buzzer, set_buzzer);	
}
void startup()
{
	/* ��������� ��������� ��������� � ������� */
	/* ����� ������ */
    /*
        PORT B:
               PB0 � MINUS BUTTON      (MODE, IN, PULLUP);
               PB1 � LIGHT	           (OC1A, OUT, 1);
               PB2 � CONTRAST          (OC1B, OUT, 1);
               PB3 � BUZZER            (OC2, OUT, 0);
               PB4 � N/A;              (N/A, OUT, 0);
               PB5 � N/A;              (N/A, OUT, 0);
               PB6 � OSC PIN           (8 MHz, IN, NO PULLUP);
               PB7 � OSC PIN           (8 MHz, IN, NO PULLUP);
        PORT C:
               PC0 � N/A;              (N/A, OUT, 0);
               PC1 � CHANNEL 2 CONTROL (CH2, OUT, 0);
               PC2 � CHANNEL 1 CONTROL (CH1, OUT, 0);
               PC3 � TEMP MEASUREMENT  (1-WIRE, OUT, 0);
               PC4 � N/A;              (N/A, OUT, 0);
               PC5 � POWER MEASUREMENT (ADC5, IN, NO PULLUP);
               PC6 � RESET PIN         (RESET, IN, NO PULLUP);
               PC7 � N/A;              (N/A, OUT, 0);
        PORT D:
			   PD0 � LCD PIN           (D7, OUT, 0);
			   PD1 � LCD PIN           (D6, OUT, 0);
			   PD2 � LCD PIN           (D5, OUT, 0);
			   PD3 � LCD PIN           (D4, OUT, 0);
			   PD4 � LCD PIN           (E, OUT, 0);
			   PD5 � LCD PIN           (RS, OUT, 0);
			   PD6 � MODE BUTTON       (MODE, IN, PULLUP);
			   PD7 � PLUS BUTTON       (MODE, IN, PULLUP);   
    */

    /* ������������� ������ */
    DDRB  = 0x3E;
	PORTB = 0x07;
    DDRC  = 0x9F;
    PORTC = 0x00;
    DDRD  = 0x3F;
    PORTD = 0xC0;

	/* ��������� �������� */
	//TCCR0A|=(0<<COM0A1)|(0<<COM0A0)|(1<<COM0B1)|(0<<COM0B0)|(0<<WGM01)|(1<<WGM00);
	// PD6 (OC0A) Output Disabled, PD5 (OC0B) Output is Phase-Correct PWM, TOP = 0xFF
	//TCCR0B|=(0<<WGM02)|(0<<CS02)|(0<<CS01)|(1<<CS00);
	// Prescaller = 1, f = 8 MHz / (1 * 510) = 15.686275 kHz
	//TCCR0B|=(0<<WGM02)|(1<<CS02)|(0<<CS01)|(1<<CS00);
	// Proteus Debug: Prescaller = 1024, f = 8 MHz / (1024 * 510) = 7.659314 Hz

	/* �������� ���������� �� EEPROM */
	eeprom_load();		

    /* ����� LCD-������� */
    /*
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01| *| T| 1| =| 9| 5| �| C|  | P| =| 9| 9| 9| 9| W|
        |02| *| T| 2| =| 9| 5| �| C|  | T| =| 0| 1| :| 4| 7|
        
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01| *| T| =| 9| 5| �| C|  | *| P| =| 9| 9| 0| 0| W|
        |02| *| U| =| 2| 3| 0| V|  | *| T| =| 0| 1| :| 5| 0| 
        
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01|  |  |  |  | S| E| T| T| I| N| G| S|  |  |  |  |
        |02|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |               
    */

    /* ������������� ������� */
    lcd_init();					// ������������� �������
    lcd_clrscr();				// ������� �������
    
    /* ����� ����������� ��� ���������� ���������� */
    lcd_goto(1, 0);
    lcd_prints("\tTo calibrate");
    lcd_goto(2, 0);
    lcd_prints("\tPress \"MODE\"");
}
void symbols_load()
{
	/* ������� ��� �������� � ������� */
	uint8_t degree[8]=
	{
		0b00110,
		0b01001,
		0b01001,
		0b00110,
		0b00000,
		0b00000,
		0b00000,
		0b00000
	};
	uint8_t thermometr[8]=
	{
		0b00100,
		0b01010,
		0b01010,
		0b01110,
		0b11111,
		0b11111,
		0b01110,
		0b00000
	};
	uint8_t flash[8]=
	{
		0b00111,
		0b01110,
		0b11100,
		0b11111,
		0b01110,
		0b11100,
		0b10000,
		0b00000
	};
	uint8_t timer[8]=
	{
		0b11111,
		0b11111,
		0b01110,
		0b00100,
		0b01110,
		0b11111,
		0b11111,
		0b00000
	};
	uint8_t plug[8]=
	{
		0b01010,
		0b01010,
		0b11111,
		0b11111,
		0b01110,
		0b00100,
		0b00010,
		0b00100
	};
    uint8_t light[8]=
    {
	    0b01110,
	    0b10001,
	    0b11011,
	    0b10101,
	    0b01110,
	    0b01110,
	    0b00100,
	    0b00000
    };	
	uint8_t contrast[8]=
	{
		0b01110,
		0b10111,
		0b10011,
		0b10011,
		0b10011,
		0b10111,
		0b01110,
		0b00000
	};
	uint8_t sound[8]=
	{
		0b00110,
		0b01010,
		0b10011,
		0b10011,
		0b10011,
		0b01010,
		0b00110,
		0b00000
	};
	
	/* �������� �������� � ������ ������� */
	lcd_load(degree, 0);		// �������� ������� ������� �� �������
	lcd_load(thermometr, 1);	// �������� ������� ����������
	lcd_load(flash, 2);			// �������� ������� ������
	lcd_load(timer, 3);			// �������� ������� �������
	lcd_load(plug, 4);			// �������� ������� �����
	lcd_load(light, 5);			// �������� ������� ��������
	lcd_load(contrast, 6);		// �������� ������� �������������
	lcd_load(sound, 7);			// �������� ������� ��������	
}
void calibrate()
{
    /* ��������� ���������� ��� ��� ���������� ����� */
    lcd_clrscr();
    lcd_goto(1, 0);
    lcd_prints(" CALIBRATING...");
    lcd_goto(2, 0);
    for(int i=0; i<NUMBER_OF_BAR_ELEMENTS; i++)
    {
        lcd_drawbar(i);
        _delay_ms(5);
    }
    lcd_clrscr();
	symbols_load();
    mode = WRK_MODE;
}
void settings()
{
    /* ��������� ��� ��������� ���������� */
    lcd_goto(1, 0);
    lcd_prints("\t\tSETTINGS");  
	lcd_goto(2, 0);
	switch(option)
    {
	    case SET_TMP:
			// ��������� �����������-���������� �����������
			lcd_prints("\t ");
			lcd_putc(1);
			lcd_prints("max: ");
			lcd_numTOstr(set_max_tmp, 2);
			lcd_putc(0);
			lcd_prints("C");
			break;
	    case SET_PWR:
			// ��������� �����������-���������� ��������
			lcd_prints("\t");
			lcd_putc(2);
			lcd_prints("max: ");
			lcd_numTOstr(set_max_pwr, 4);
			lcd_prints(" W");
			break;
	    case SET_VTG:
			// ��������� ���������� ����
			lcd_prints("\t ");
			lcd_putc(4);
			lcd_prints("in: ");
			lcd_numTOstr(set_vtg, 3);
			lcd_prints(" V");
			break;
	    case SET_TIM:
			// ��������� ������� ���������� ����������� (����������� �������� �����������)
			lcd_prints("\t\t");
			lcd_putc(3);
			lcd_prints(": ");
			lcd_numTOstr(set_timer / 60, 2);
			lcd_prints(":");
			lcd_numTOstr(set_timer % 60, 2);
			break;
	    case SET_BUZ:
			// ��������� ��������� ����������
			lcd_prints("\t\t ");
			lcd_putc(7);
			lcd_prints(": ");
			if(set_buzzer)
				lcd_prints("ON ");
			else
				lcd_prints("OFF");
			break;
		case SET_LUM:
			// ��������� ������� �������
			lcd_prints("\t\t ");
			lcd_putc(5);
			lcd_prints(": ");
			lcd_numTOstr(set_light, 3);
			break;
		case SET_CTR:
			// ��������� ������������� �������
			lcd_prints("\t\t ");
			lcd_putc(6);
			lcd_prints(": ");
			lcd_numTOstr(set_contrast, 3);
			break;						
    }	
}
void temp_out(uint8_t channel, uint8_t temp_value, uint8_t status)
{
    /* ����� ��� ������ �������� ����������� �� ������� */
    lcd_goto(channel, 0);
    if(status)        
		lcd_prints(">");
    else
        lcd_prints(" ");
    lcd_putc(1);
	lcd_itostr(channel);
    lcd_prints("=");
    lcd_numTOstr(temp_value, 2);
    lcd_putc(0);
    lcd_prints("C");
}
void power_out(uint16_t power_value)
{
    /* ����� ��� ������ �������� �������� �� ������� */
    lcd_goto(1, 9);
    lcd_putc(2);
	lcd_prints("=");
	lcd_numTOstr(power_value, 4);
    lcd_prints("W");
}
void time_out(uint8_t set_timer, uint8_t timer_value)
{
    /* ����� ��� ������ ����������� ������� �� ������� */
    uint8_t time = set_timer - timer_value;
    uint8_t hours = time / 60;
    uint8_t mins = time % 60;
    lcd_goto(2, 9);
    lcd_putc(3);
	lcd_prints("=");
    lcd_numTOstr(hours, 2);  
    lcd_prints(":");
    lcd_numTOstr(mins, 2);
}
void display()
{
	/* ����� ������ ������ �� �������� ����� */
	temp_out(1, 27, False);
	temp_out(2, 32, True);
	power_out(9900);
	if(set_timer != 0)
		time_out(set_timer, timer_value);
	// ������ = 85 ��� = 1:25, ������ = 0:13, �������� = 1:12
}
void buttons_check()
{
    /* ��������� �������� ������� ������ */
	if(CHECKBIT(PIND, MODE_BUTTON) == 0)
    {
        _delay_ms(DEB_INT);
        if(launch == True)
        {
            launch = False;
            mode = CAL_MODE;
        }
        else
        {
            lcd_clrscr();
            if(mode == SET_MODE)
            {
				if(option < SET_CNT-1)
                    option++;
                else
                {
					eeprom_save();
					option = SET_TMP;
                    mode = WRK_MODE;
                }
            }
            else
            {
                if(mode < MODE_CNT-1)
                    mode++;
                else
					mode = WRK_MODE;
            }
        }
    }
	if(mode == SET_MODE)
	{
		if(CHECKBIT(PIND, PLUS_BUTTON) == 0)
		{
			_delay_ms(DEB_INT);
			switch(option)
			{
				case SET_TMP:
 					if(set_max_tmp < temp_max)
 						set_max_tmp += temp_delta;
					break;
				case SET_PWR:
					if(set_max_pwr < power_max)
						set_max_pwr += power_delta;
					break;
				case SET_VTG:
					if(set_vtg < voltage_max)
						set_vtg += voltage_delta;
					break;
				case SET_TIM:
					if(set_timer < timer_max)
						set_timer += timer_delta;
					break;
				case SET_BUZ:
					set_buzzer = ~set_buzzer;
					break;
				case SET_LUM:
					if(set_light < lcd_light_max)
						set_light += lcd_light_delta;
					break;					
				case SET_CTR:
					if(set_contrast < lcd_contr_max)
						set_contrast += lcd_contr_delta;
					break;											
			}
		}
		else if(CHECKBIT(PINB, MINUS_BUTTON) == 0)
		{
			_delay_ms(DEB_INT);
			switch(option)
			{
				case SET_TMP:
					if(set_max_tmp > temp_min)
						set_max_tmp -= temp_delta;
					break;
				case SET_PWR:
					if(set_max_pwr > power_min)
						set_max_pwr -= power_delta;
					break;
				case SET_VTG:
					if(set_vtg > voltage_min)
						set_vtg -= voltage_delta;
					break;
				case SET_TIM:
					if(set_timer > timer_min)
						set_timer -= timer_delta;
					break;
				case SET_BUZ:
					set_buzzer = ~set_buzzer;
					break;
				case SET_LUM:
					if(set_light > lcd_light_min)
						set_light -= lcd_light_delta;
					break;					
				case SET_CTR:
					if(set_contrast > lcd_contr_min)
						set_contrast -= lcd_contr_delta;
					break;
			}
		}		
	}
	else if(SET_EASTER_EGG)
	{
		if((CHECKBIT(PIND, PLUS_BUTTON) == 0) && (CHECKBIT(PINB, MINUS_BUTTON) == 0))
			SET_CNT = SET_CNT_MAX;
	}
}

int main(void)
{
    startup();
    while(1)
    {
        buttons_check();
        switch(mode)
        {
            case WRK_MODE:
				display();
                break;
            case SET_MODE:
				settings();
                break;
            case CAL_MODE:
                calibrate();
                break;            
        }
		CONTRAST = set_contrast;
    }
    return 0;
}