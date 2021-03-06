/*
 * TP-Meter.c
 *
 * Created:     21.08.2015 16:36:53
 * CPU:         ATMega8
 * LCD:         WH1602
 * Frequency:   16 MHz
 * Author:      Symrak
 *
 */

/* ����������� ������� ������� �� */
#define F_CPU 16000000

/* ����������� ��������� */
/* ����������� ���������� */
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/interrupt.h>
/* ���������� ��� HD44780 ������� */
#include "hd44780.h"
/* ��������� ��� ������ � ����� 1-Wire */
#include "OWIPolled.h"
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"
#include "OWIcrc.h"

/* ����������� ����. �������� */
#define True            1
#define False           0
#define On				1
#define Off				0

/* ����������� ��� ���� 1-Wire */
/* ��� ��������� � ���� ������ DS18B20 */
#define DS18B20_FAMILY_ID           0x28
#define DS18B20_CONVERT_T           0x44
#define DS18B20_READ_SCRATCHPAD     0xBE
#define DS18B20_WRITE_SCRATCHPAD    0x4E
#define DS18B20_COPY_SCRATCHPAD     0x48
#define DS18B20_RECALL_E            0xB8
#define DS18B20_READ_POWER_SUPPLY   0xB4
#define RES_9BIT					0x1F
/* ������������ ������ ��� ���� 1-Wire */
#define BUS							OWI_PIN_3
/* ���������� ��������� �� ���� 1-Wire */
#define MAX_DEVICES					2
/* ���� ������ ��� ������� ������ ����������� */
#define READ_SUCCESSFUL				0x00
#define READ_CRC_ERROR				0x01
#define SEARCH_SENSORS				0x00
#define SENSORS_FOUND				0xFF
/* ���������� ��� ������������� ���� 1-Wire */
/* ������ �������� ��� �������� ������� */
OWI_device allDevices[MAX_DEVICES];
/* ����� ���������� */
unsigned char rom[8];
/* ����� ��� ���� 1-Wire */
unsigned char searchFlag          = SEARCH_SENSORS;
unsigned char crcFlag             = 0;
uint8_t dev_searching             = True;
/* ������� ���������� ��������� �� ���� 1-Wire */
unsigned char dev_num             = 0;

/* ����������� ������ */
#define MODE_BUTTON     PD6
#define PLUS_BUTTON     PD7
#define MINUS_BUTTON    PB0

/* ����������� ���� ��� EasterEgg */
#define EGG				PC5

/* ����������� ������� �������� (��) ��� ���������� �������� ��������� */
#define DEB_INT		    50

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
#define SET_HM			4
#define SET_BUZ	        5
#define SET_LUM		    6
#define SET_CTR	        7

/* ����������� ������ � ���� ��� ������������� �������� */
/* ������� � ��� ������������ ����������� */
#define temp_min        20
#define temp_max        85
#define temp_delta      5
/* ������� � ��� ������������ ������������ �������� */
#define power_min       100
#define power_max       9900
#define power_delta     50
/* ������� � ��� �������� �������� �������� ���������� */
#define voltage_min     180
#define voltage_max     250
#define voltage_delta   1
#define voltage_default 230
/* ������� � ��� ������� ���������� �������� */
#define timer_min       0
#define timer_max       7200 // �������� � �������� = 120 �����
#define timer_delta     60	 // �������� � �������� = 1 ������
#define timer_default   0
/* ������� � ��� �������� ������� ������� */
#define lcd_light_min   0
#define lcd_light_max   255
#define lcd_light_delta 5
/* ������� � ��� �������� ������������� ������� */
#define lcd_contr_min   0
#define lcd_contr_max   255
#define lcd_contr_delta 5
#define lcd_contr_def   100
/* ������� � �������� ��-��������� ��� ������ �������� */
#define heating_mode_min 0
#define heating_mode_max 2
#define heating_mode_def 0

/* ���������� ���������� ���������� */
/* ���������� ������� � ������� ������ */
uint8_t mode          = 0;
uint8_t option        = 0;
uint8_t launch        = True;
/* ����������, �������� �������� �������� */
uint8_t           set_max_tmp  = 0;
uint8_t  EEMEM ee_set_max_tmp  = 0;
uint16_t          set_max_pwr  = 0;
uint16_t EEMEM ee_set_max_pwr  = 0;
uint8_t           set_vtg      = 0;
uint8_t EEMEM  ee_set_vtg      = 0;
uint16_t		  set_timer    = 0;
uint16_t EEMEM ee_set_timer    = 0;
uint8_t           set_light    = 0;
uint8_t EEMEM  ee_set_light    = 0;
uint8_t           set_contrast = 0;
uint8_t EEMEM  ee_set_contrast = 0;
uint8_t           set_buzzer   = True;
uint8_t EEMEM  ee_set_buzzer   = True;
uint8_t           heating_mode = 0;
uint8_t EEMEM  ee_heating_mode = 0;
/* ����������� ������� ������� ���������� */
uint8_t EEMEM  ee_first_run    = False;

/* ����������� ��������� �������� ��� ������ ��� */
#define LIGHTNESS       OCR1A
#define CONTRAST        OCR1B

/* ����������� �������� �������� ��� Buzzer */
#define BUZZER	        TCCR2

/* ����������� ������� ���������� ��������� */
#define LOAD_PORT		PINC
#define CH1				PC2
#define CH2				PC1

/* ���������� ��� ������� ������� */
/* ������� ���������� ������� �0 */
uint16_t timer_counter = 0;
/* ���������� ������� ��������� ������� */
uint8_t timer_enable  = False;
uint8_t timer_out     = False;
uint16_t timer_secs   = 0;

/* ���������� ������ ������ ��������� ���������� */
#define BUZZ_SIG_CNT    5
uint8_t BUZZED_TIMES  = 0;
uint8_t BUZZ_PART     = 0;
uint8_t BUZZ_CFG      = 0;

/* �������� ���������� ������� � ����� �������� */
uint8_t CH1_temp      = 0;
uint8_t CH2_temp	  = 0;
uint16_t cur_power    = 0;

/* ���������� ������ ��������� ����������� */
uint8_t temp_flag	  = False;
uint8_t temp_timer    = 1;
#define temp_timer_max  5

/* ���� ���������� ������������ �������� */
uint8_t overpower	  = False;

/* ����������, ����������� ��� ������ ��� */
unsigned long adc_value   = 0;
uint16_t      adc_noise   = 0;
#define NOISE_CAL_CNT 500
#define POWER_GET_CNT 100

/* ����� ���������� �������� � ���������� ��������� �������� */
uint8_t enable_eegg	  =	False;
#define SET_EASTER_EGG	True
#define SET_CNT_MAX		8
uint8_t SET_CNT       = 6;

/* ���������� ��� ������� */
#define PROTEUS       False
#define FULL_MENU     False

/* ������������ � ��������� */
#define X             30
#define VREF		  2.5
#define ADC_RES		  1024

void eeprom_load()
{
	/* ��������� �������� �������� �� EEPROM */
	/* ����������� ������� ������� */
	uint8_t first_run = eeprom_read_byte(&ee_first_run);
	if(first_run != False)
		eeprom_write_byte(&ee_first_run, False);
	/* �������� �������� �� EEPROM */
	set_max_tmp = eeprom_read_byte(&ee_set_max_tmp);
	if((set_max_tmp < temp_min) || (set_max_tmp > temp_max))
		set_max_tmp = temp_min;
	
	set_max_pwr = eeprom_read_word(&ee_set_max_pwr);
	if((set_max_pwr < power_min) || (set_max_pwr > power_max))
		set_max_pwr = power_min;
	
	set_vtg = eeprom_read_byte(&ee_set_vtg);
	if((set_vtg < voltage_min) || (set_vtg > voltage_max))
		set_vtg = voltage_default;
	
	set_timer = eeprom_read_word(&ee_set_timer);
	if((set_timer < timer_min) || (set_timer > timer_max))
		set_timer = timer_default;

	set_light = eeprom_read_byte(&ee_set_light);
		if((set_light < lcd_light_min) || (set_light > lcd_light_max))
			set_light = lcd_light_max;

	set_contrast = eeprom_read_byte(&ee_set_contrast);
	if((set_contrast < lcd_contr_min) || (set_contrast > lcd_contr_max) || (first_run == 0xFF))
		set_contrast = lcd_contr_def;
	
	set_buzzer = eeprom_read_byte(&ee_set_buzzer);
	if((set_buzzer < 0) || (set_buzzer >= 1))
		set_buzzer = True;
	else
		set_buzzer = False;
	
	heating_mode = eeprom_read_byte(&ee_heating_mode);
		if((heating_mode < heating_mode_min) || (heating_mode > heating_mode_max))
			heating_mode = heating_mode_def;
}
void eeprom_save()
{
	/* ��������� ���������� �������� � EEPROM */
	eeprom_write_byte(&ee_set_max_tmp, set_max_tmp);
	eeprom_write_word(&ee_set_max_pwr, set_max_pwr);
	eeprom_write_byte(&ee_set_vtg, set_vtg);
	eeprom_write_word(&ee_set_timer, set_timer);
	eeprom_write_byte(&ee_set_light, set_light);
	eeprom_write_byte(&ee_set_contrast, set_contrast);
	eeprom_write_byte(&ee_set_buzzer, set_buzzer);
	eeprom_write_byte(&ee_heating_mode, heating_mode);
}
void symbols_load()
{
	/* ��������� �������� ����������� �������� � ������� */
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
void load_control(uint8_t channel, uint8_t state)
{
	/* ��������� ��������� ���������.
	   ���� ����������� ������������� ��������,
	   � ����� � ���� ���� �� ���� ����� 0,
	   �� �������� �������� � ��������.        */
	switch (channel)
	{
		case 1:
			if(state && ((heating_mode == 0) || (heating_mode == 1)))
				DISABLE(PORTC, CH1);
			else
				ENABLE(PORTC, CH1);
			break;
		case 2:
			if(state && ((heating_mode == 0) || (heating_mode == 2)))
				DISABLE(PORTC, CH2);
			else
				ENABLE(PORTC, CH2);
			break;
		default:
			if(state && (heating_mode == 0))
			{
				DISABLE(PORTC, CH1);
				DISABLE(PORTC, CH2);
			}
			else
			{
				ENABLE(PORTC, CH1);
				ENABLE(PORTC, CH2);				
			}
			break;
	}
}
void ds18b20_search()
{
    /* ��������� ������ �������� ����������� */
	/* ���� ���� ������� - ��������� ����� 1-Wire ���������.
       ���� ���-�� �������� �������� ��������� � ���-��� ��������� -
	   ������������� ����, ����� ������� ������ ������ �� �����������. */
    if (searchFlag == SEARCH_SENSORS)
	{
		dev_num = 0;
		crcFlag = OWI_SearchDevices(allDevices, MAX_DEVICES, BUS, &dev_num);
		if((dev_num == MAX_DEVICES) && (crcFlag != SEARCH_CRC_ERROR))
			searchFlag = SENSORS_FOUND;
    }	
}
void ds18b20_resolution_set(unsigned char bus, unsigned char * id, uint8_t resolution_code)
{
	OWI_DetectPresence(bus);
	OWI_MatchRom(id, bus);
	OWI_SendByte(DS18B20_WRITE_SCRATCHPAD, bus);
	OWI_SendByte(0, bus);
	OWI_SendByte(0, bus);
	OWI_SendByte(resolution_code, bus);
}
unsigned char ds18b20_read_temp(unsigned char bus, unsigned char * id, unsigned int* temperature)
{
    /* ��������� ������ ����������� � ������� */
	unsigned char scratchpad[9];
    unsigned char i;
    /* ������ ������ ������.
	   ������ ������� ���������� 1-Wire ���������� �� ����.
       ������ ������� ������� ��������������. */
    OWI_DetectPresence(bus);
    OWI_MatchRom(id, bus);
    OWI_SendByte(DS18B20_CONVERT_T, bus);
    /* ���� ���������� �������������� */ 
    while(!OWI_ReadBit(bus));
    /* ������ ������ ������.
	   ������ ������� ��� ���������� 1-Wire ���������� �� ����.
       ������ ������� ������ ���������� ������.
       ��������� ���������� ������ ������� � ������. */	
    OWI_DetectPresence(bus);
    OWI_MatchRom(id, bus);
    OWI_SendByte(DS18B20_READ_SCRATCHPAD, bus);
    for(i=0; i<=8; i++)
		scratchpad[i] = OWI_ReceiveByte(bus);
    if(OWI_CheckScratchPadCRC(scratchpad) != OWI_CRC_OK)
		return READ_CRC_ERROR;
    
    *temperature =(unsigned int)scratchpad[0];
    *temperature|=((unsigned int)scratchpad[1]<<8);
    
    return READ_SUCCESSFUL;
}
uint8_t ds18b20_get_temp(uint8_t dev_id)
{
	/* ��������� ����������� � �������� ����������� � ���������� ������� */
    /* ���������� ��� �������� "�����" ����������� � ������� */
    unsigned int temperature = 0;
    /* ���������� ��� ����������� ����������� */
    uint8_t tmp = 0;
    /* ����� ������� */
    crcFlag = ds18b20_read_temp(BUS, allDevices[dev_id].id, &temperature);
	if(crcFlag != READ_CRC_ERROR)
	{
        /* ���� ����������� �������������, ��������� �������������� */
        if((temperature & 0x8000) != 0)
            temperature = ~temperature + 1;
        /* ����� ����� ����������� */
        tmp = (uint8_t)(temperature>>4);
        /* ������� ����� ����������� */
        // 	tmp = (unsigned char)(temperature&15);
        // 	tmp = (tmp>>1) + (tmp>>3);
    }  
    return tmp;
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
               PB4 � N/A               (N/A, OUT, 0);
               PB5 � N/A               (N/A, OUT, 0);
               PB6 � OSC PIN           (16 MHz, IN, NO PULLUP);
               PB7 � OSC PIN           (16 MHz, IN, NO PULLUP);
        PORT C:
               PC0 � N/A               (N/A, OUT, 0);
               PC1 � CHANNEL 2 CONTROL (CH2, OUT, 1);
               PC2 � CHANNEL 1 CONTROL (CH1, OUT, 1);
               PC3 � TEMP MEASUREMENT  (1-WIRE, OUT, 0);
			   PC4 � POWER MEASUREMENT (ADC4, IN, NO PULLUP);
			   PC5 � EASTER EGG        (EGG, IN, PULLUP);
               PC6 � RESET PIN         (RESET, IN, NO PULLUP);
               PC7 � N/A               (N/A, OUT, 0);
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
    DDRC  = 0x8F;
    PORTC = 0x26;
    DDRD  = 0x3F;
    PORTD = 0xC0;
	
	/* ��������� �������� */
	TIMSK|=(1<<TOIE0);
	// ���������� ���������� �� ������������ ������� T0
	TCCR0|=(0<<CS02)|(0<<CS01)|(1<<CS00);
	// Prescaller: N = 1; f = 16 MHz 
	TCCR1A|=(1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(1<<WGM10);
	// PB1 (OC1A = LIGHT) � PB2 (OC1B = CONTRAST) ������ Phase-Correct PWM, 8-bit, TOP = 0x00FF
	TCCR1B|=(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10);
	// Prescaller: N = 1, f = 16 MHz / (2 * 1 * 256) = 31.250000 kHz
	TCCR2|=(1<<COM21)|(0<<COM20)|(1<<WGM21)|(1<<WGM20)|(1<<CS22)|(0<<CS21)|(0<<CS20);
	// PB3 (OC2 = BUZZER) ������ Fast PWM, TOP = 0xFF, Prescaller: N = 64, f = 16 MHz / (64 * 256) = 976.562500 Hz
	OCR2=0x7F;
	/*		f = 976.5625 Hz = 0.9765625 kHz
			T = 1/f = 1/0.9765625 = 1.024 s
			S = T/t = TOP/OCR2 = 255/127 = 2.007874
			t = T/S = 1.024/2.007874 = 0.509992 s
	*/
	
	/* ��������� ��� */
	ADMUX|=(0<<REFS1)|(0<<REFS0)|(0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(0<<MUX0);
	// ���: ������� �� AREF, TL431 (2.50 V); ���� ���: ADC4
	ADCSRA|=(0<<ADEN)|(0<<ADSC)|(0<<ADFR)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	// ��������� �� �������, ���������� �� ��������� ��������������, �������� 128: f ADC = 16 MHz / 128 = 125 kHz
	
	/* ���� ��� ������� */
		if(PROTEUS)
        {
            TCCR1B|=(0<<WGM13)|(0<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10);
            // Prescaller: N = 1024, f = 16 MHz / (2 * 1024 * 256) = 30.517578 Hz
            TCCR2|=(1<<COM21)|(0<<COM20)|(1<<WGM21)|(1<<WGM20)|(1<<CS22)|(1<<CS21)|(1<<CS20);
            // Prescaller: N = 1024, f = 16 MHz / (1024 * 256) = 61.035156 Hz            
        }
		if(FULL_MENU)
            SET_CNT = SET_CNT_MAX;
	/* ���� ��� ������� */
	
	/* ���������� �������� ������� OC2 ��� Buzzer */
	BUZZ_CFG = BUZZER;
	
	/* ���������� Buzzer */
	BUZZER = 0;

	/* �������� ���������� ����������� ������������ Easter Egg */
	if(CHECKBIT(PINC, EGG) == 0)
		enable_eegg = True;
	else
		enable_eegg = False;

	/* �������� ���������� �� EEPROM */
	eeprom_load();		

    /* ���������� ���������� ������� � ��������� */
    LIGHTNESS = set_light;
    CONTRAST  = set_contrast;

    /* ����� LCD-������� */
    /*
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01| *| T| 1| =| 9| 5| �| C|  | P| =| 9| 9| 9| 9| W|
        |02| *| T| 2| =| 9| 5| �| C|  | T| =| 0| 1| :| 4| 7|
        
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01|  |  |  |  | S| E| T| T| I| N| G| S|  |  |  |  |
        |02|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |               
    */

    /* ������������� ������� */
    lcd_init();
    lcd_clrscr();

	/* ����� ���������� � ������ �������� */
	lcd_goto(1, 0);
	lcd_prints("\tTemp. sensors");
	lcd_goto(2, 0);
	lcd_prints("\tdetecting...");

	/* ������������� ���� 1-Wire */
	OWI_Init(BUS);
    
    /* ����� �������� �� ���� */
    while(searchFlag != SENSORS_FOUND)
        ds18b20_search();
    dev_searching = False;
    
	/* ��������� ���������� �������� */
	ds18b20_resolution_set(BUS, allDevices[0].id, RES_9BIT);
	ds18b20_resolution_set(BUS, allDevices[1].id, RES_9BIT);
	
	/* ��������� ����������� � �������� */
	CH1_temp = ds18b20_get_temp(0);
	CH2_temp = ds18b20_get_temp(1);
	
	/* ����� ����������� ��� ���������� ���������� */
	lcd_clrscr();
    lcd_goto(1, 0);
	lcd_prints("\tTo calibrate");
	lcd_goto(2, 0);
	lcd_prints("\tPress \"MODE\"");	
    
    /* ���������� ���������� ����������:
			� ������� asm("sei") �������� � ��������� calibrate(),
			  �.�. ��� �� "������" ������.
	*/
}
void settings()
{
    /* ��������� ��������� ���������� */
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
			lcd_numTOstr(set_timer/3600, 2);
			lcd_prints(":");
			lcd_numTOstr((set_timer/60)%60, 2);
			break;
		case SET_HM:
			// ��������� ������� �������� (���������� ����������)
			lcd_prints("\t\t M: ");
			switch(heating_mode)
			{
				case 0:
					// �������� ��� ������
					lcd_prints("1&2");
					break;
				case 1:
					// ������� ������ ������ �����
					lcd_prints("1  ");
					break;
				case 2:
					// ������� ������ ������ �����
					lcd_prints("2  ");
					break;
			}	
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
void buzz(uint8_t state)
{
	/* ��������� ���������/���������� �������������� ������� (Buzzer) */
	if(state)
		BUZZER = BUZZ_CFG;
	else
		BUZZER = 0;
}
void timer_reset()
{
	/* ��������� ������ ������� */
	timer_enable = False;
	timer_out = False;
	timer_secs = 0;	
}
uint16_t get_adc_value(uint16_t count)
{
	/* ��������� ��������� �������� � ��� */
	uint8_t cal_mode = False;
	unsigned int adc_values_accumulator = 0;
	uint16_t bar_count = count/NUMBER_OF_BAR_ELEMENTS;
	/* �������� ����� ������ ���������� */
	if(mode == CAL_MODE)
		cal_mode = True;
	/* ���� ������ � ��� � ����� ����� ��������� (� ������ ������ ����������) */
	for(uint16_t i=0; i<count; i++)
	{
		ADCSRA|=(1<<ADSC);
		if(!cal_mode)
			_delay_ms(1);
		adc_values_accumulator += adc_value;
		if(cal_mode)
			lcd_drawbar(i/bar_count);
	}
	/* ����������� �������� */
	adc_values_accumulator /= count;
	return (uint16_t)adc_values_accumulator;
}
uint16_t get_power_value()
{
	/* ������� �������� "��������-���" ��� ������� SCT-013-000:
	      50 mV => 100 A
	       2 mV => 1 A
		   1 mV => 0.5 A
	     0.1 mV => 0.05 A
		  U(mV) =  I(A) * 0.5
	  
	  ������ ���:
		BITs = 1024
		Vref = 2.5 V
		 ADC = ((Vin * 1024) / Vref)
		  dV = 2.5 / 1024 = 2.441406 mV / bit => 0.002441 V / bit
	*/
	
	/* �������� �������� ���������� � ��� � �������� �� ���� ��� */
    uint16_t power_value = get_adc_value(POWER_GET_CNT);
    if(power_value > adc_noise)
        power_value -= adc_noise;
    else
        power_value = 0;
	/* ��������� ���������� �������� � ���������� ������� � ��������� �������� ������������ ��������*/
	power_value = (power_value*VREF/ADC_RES)*VREF*X*set_vtg;
	/* ���������� ���������� �������� */
	return power_value;
}
void values_refresh()
{
	/* ��������� ���������� �������� ����������, ��� �� ���������.
	   ��������, ��� �� ��������� �������� ��� ���������� ����������� ������/���������� ��������. */
	/* ��������� �������� ������������ �������� */
    cur_power = get_power_value();
	/* ��������� ���������� �������� ��� ���������� ������������ �������� */
	if(cur_power >= set_max_pwr)
	{
		if(!overpower)
		{
			lcd_clrscr();
			overpower = True;
			timer_reset();
			load_control(0, Off);
			buzz(On);
		}
	}
	/* ��������� ���������� ������� */
	if(temp_flag)
	{
		temp_flag = False;
		CH1_temp = ds18b20_get_temp(0);
		CH2_temp = ds18b20_get_temp(1);
	}	
	/* ���������� �������� �������� � ����������� �� ����������� � �������� */
	if(!overpower)
	{
		/* ���� ����� ������� = 0 (��������) ��� ��� �� ������� (��������) */
		if((set_timer == 0) || (timer_enable))
		{
			if(CH1_temp < set_max_tmp)
            {
                if(CH1_temp != 0)
				    load_control(1, On);
                else
                    load_control(1, Off);                    
            }			
            else
				load_control(1, Off);
			if(CH2_temp < set_max_tmp)
            {
                if(CH2_temp != 0)
				    load_control(2, On);
                else
                    load_control(2, Off);
            }			
            else
				load_control(2, Off);
		}
	}
	else
		load_control(0, Off);
	/* ���������� �������� ������� � ��������� ��� �������� */
	if(mode == SET_MODE)
    {
        if(LIGHTNESS != set_light)
            LIGHTNESS = set_light;
        if(CONTRAST != set_contrast)
            CONTRAST = set_contrast;       
    }	
}
void temp_out(uint8_t channel, uint8_t temp_value, uint8_t status)
{
    /* ��������� ������ �������� ����������� �� ������� */
    lcd_goto(channel, 0);
    if(status)        
		lcd_prints(" ");
    else
        lcd_prints(">");
    lcd_putc(1);
	lcd_itostr(channel);
    lcd_prints("=");
    lcd_numTOstr(temp_value, 2);
    lcd_putc(0);
    lcd_prints("C");
}
void power_out(uint16_t power_value)
{
    /* ��������� ������ �������� �������� �� ������� */
    lcd_goto(1, 9);
    lcd_putc(2);
	lcd_prints("=");
	lcd_numTOstr(power_value, 4);
    lcd_prints("W");
}
void time_out(uint16_t set_timer, uint16_t timer_secs)
{
    /* ��������� ������ ����������� ������� �� ������� */
	if(timer_enable && !timer_out)
	{
		uint16_t time  = set_timer-timer_secs;
		uint16_t tmp   = (time/3600)*3600;
		uint8_t  first = 0;
		uint8_t second = 0;
		if(time < 3600)
		{
			first  = (time-tmp)/60;		  // ������
			second = time-tmp-(first*60); // �������
		}
		else
		{
			first  = time/3600;		      // ����
			second = (time-tmp)/60;	      // ������
		}
		lcd_goto(2, 9);
		lcd_putc(3);
		lcd_prints("=");
		lcd_numTOstr(first, 2);
		if((timer_secs % 2) == 0)
			lcd_prints(":");
		else
			lcd_prints(" ");
		lcd_numTOstr(second, 2);
	}
	else if(timer_out)
	{
		lcd_goto(2, 9);
		lcd_putc(3);
		lcd_prints("=--:--");	
	}
	else
	{
		lcd_goto(2, 9);
		lcd_prints("       ");		
	}
}
void display()
{
	/* ��������� ������ ������ �� �������� ����� */
	if(!overpower)
	{
		temp_out(1, CH1_temp, CHECKBIT(LOAD_PORT, CH1));
		temp_out(2, CH2_temp, CHECKBIT(LOAD_PORT, CH2));
		power_out(cur_power);
		time_out(set_timer, timer_secs);
	}
	else
	{
		lcd_goto(1, 0);
		lcd_prints("W A R N I N G!!!");
		lcd_goto(2, 0);
		lcd_prints("\tOVERPOWER!!!");
	}
}
void calibrate()
{
	/* ��������� ���������� ��� ��� ���������� ����� */
	/* ��������� ��� */
	ADCSRA|=(1<<ADEN);
	/* ������ ������� �������������� ��� */
	ADCSRA|=(1<<ADSC);
	/* ����� ���������� */
	lcd_clrscr();
	lcd_goto(1, 0);
	lcd_prints(" CALIBRATING...");
	lcd_goto(2, 0);
	/* ���������� ���������� ���������� */
	sei();
	/* ���������� �������� ���� */	
	adc_noise = get_adc_value(NOISE_CAL_CNT);
	/* ������� ������� ����� �������� �������� */
	lcd_clrscr();
	/* ���������� ������ ���������� */
	cli();
	/* �������� �������� � ������� */
	symbols_load();
	/* ���������� ���������� ���������� */
	sei();
	/* ������� � ������� ����� */
	lcd_clrscr();
	mode = WRK_MODE;
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
					if(set_timer != 0)
					{
						timer_reset();
						timer_enable = True;
					}
					else
						if(timer_enable)
							timer_reset();
					if(overpower)
					{
						buzz(Off);
						overpower = False;
					}
					mode = WRK_MODE;
                }
            }
            else
            {
                if(mode < MODE_CNT-1)
				{
					if(overpower)
						buzz(Off);                  
					mode++;					
				}
				else
				{
					mode = WRK_MODE;
				}
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
				case SET_HM:
					if(heating_mode < heating_mode_max)
						heating_mode++;
					else
						heating_mode = heating_mode_min;
					break;
				case SET_BUZ:
					set_buzzer ^= On;
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
					if(set_timer == timer_min)
					{
						timer_reset();
						buzz(Off);
					}
					break;
				case SET_HM:
					if(heating_mode > heating_mode_min)
						heating_mode--;
					else
						heating_mode = heating_mode_max;
					break;			
				case SET_BUZ:
					set_buzzer ^= On;
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
	else if(SET_EASTER_EGG && enable_eegg)
	{
		if(CHECKBIT(PINC, EGG) != 0)
		{
			if((mode == WRK_MODE) && (CHECKBIT(PIND, PLUS_BUTTON) == 0) && (CHECKBIT(PINB, MINUS_BUTTON) == 0))
				SET_CNT = SET_CNT_MAX;
		}
	}
}

ISR(TIMER0_OVF_vect)
{
	/* ������ ���������� �� ������������ ������� �0 */
    /* ������� ������������ "�����-������������": 
          2 ��    => 125
          10 ��   => 625
          50 ��   => 3125
          100 ��  => 6250
          200 ��  => 12500
          250 ��  => 15625
          500 ��  => 31250
          1000 �� => 62500
    */
	timer_counter++;
	/* �������� �� ���������� ����� ������� */
	if(timer_counter>=62500)
	{
		/* ���������� ��� ������� ��������� ������� */
		if(timer_enable)
		{
			/* ���������� �������� ������ */
			timer_secs++;
			if(timer_secs >= set_timer)
			{
				/* �������� ��� ���������� ������� ��������� ������� */
				/* ��������� ������ ��������� �������, ������� �������� ����� � ������ */
				timer_reset();
				/* ��������� ��� �������� */
				load_control(0, Off);
				/* �������� ������, ��� ������������ */
				timer_out = True;	
			}			
		}
		/* ���������� ��� ������ ��������� ������� */
		if(set_buzzer && timer_out)
		{
			if(BUZZED_TIMES<BUZZ_SIG_CNT)
			{
				if(BUZZ_PART == 0)
				{
					buzz(On);
					BUZZ_PART++;
				}
				else
				{	
					buzz(Off);
					BUZZ_PART = 0;
					BUZZED_TIMES++;
				}
			}
			else
			{
				timer_out = False;
				BUZZED_TIMES = 0;  
			}
		}
		else
			timer_out = False;
		/* ���������� �������� ������� ������� ����� ����������� ����������� */
		if(temp_timer < temp_timer_max)
			temp_timer++;
		else
		{
			temp_timer = 1;
			temp_flag  = True;
		}			
		/* ��������� �������� ���������� ������� �0 */
		timer_counter = 0;
	}
}
ISR(ADC_vect)
{
    adc_value = ADC;
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
		if((mode != 0) && (mode != CAL_MODE))
			values_refresh();
    }
    return 0;
}