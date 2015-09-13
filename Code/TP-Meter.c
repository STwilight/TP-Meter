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

/*
	TODO List:
		– проверить работу values_refresh() для настройки ШИМ в железе
		– проверить работу load_control() в железе
		– проверить работу таймера обратного отсчета в железе
		– проверить работу процедуры генерации звука в железе
		– проверить работу опроса датчиков по прерыванию таймера в железе
		– проверить работу процелуры управления каналами в зависимости от настроек времени таймера обратного отсчета в железе
		– проверить работу процедуры аварийного отключения каналов нагрузки в случае превышения мощности в железе
		– проверить работу процедуры управления каналом нагрузки в зависимости от температуры на нем в железе
		– проверить верное определение первого запуска устройства в железе
		– проверить работу Easter Egg в железе
		– проверить работу аппаратной блокировки включения Easter Egg в железе
		
		– написать функцию конвертации полученного значения в необходимую величину и вывести ее на экран
		– проверить работу АЦП в железе
		
		– разобраться с принципом определения номера датчика
		– проверить работу в железе

*/

/* Определение рабочей частоты МК */
#define F_CPU 8000000

/* Подключение библиотек */
#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "hd44780.h"
/* Билиотеки для работы с шиной 1-Wire */
#include "OWIPolled.h"
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"
#include "OWIcrc.h"

/* Определение спец. значений */
#define True            1
#define False           0
#define On				1
#define Off				0

/* Определения для шины 1-Wire */
/* Код семейства и коды команд DS18B20 */
#define DS18B20_FAMILY_ID           0x28
#define DS18B20_CONVERT_T           0x44
#define DS18B20_READ_SCRATCHPAD     0xBE
#define DS18B20_WRITE_SCRATCHPAD    0x4E
#define DS18B20_COPY_SCRATCHPAD     0x48
#define DS18B20_RECALL_E            0xB8
#define DS18B20_READ_POWER_SUPPLY   0xB4
/* Опредеделние вывода под шину 1-Wire */
#define BUS							OWI_PIN_3
/* Количество устройств на шине 1-Wire */
#define MAX_DEVICES					2
/* Коды ошибок для функции чтения температуры */
#define READ_SUCCESSFUL				0x00
#define READ_CRC_ERROR				0x01
#define SEARCH_SENSORS				0x00
#define SENSORS_FOUND				0xFF
/* Переменные для использования шины 1-Wire */
/* Массив структур для хранения адресов */
OWI_device allDevices[MAX_DEVICES];
/* Адрес устройства */
unsigned char rom[8];
/* Флаги для шины 1-Wire */
unsigned char searchFlag          = SEARCH_SENSORS;
unsigned char crcFlag             = 0;
/* Текущее количество устройств на шине 1-Wire */
unsigned char dev_num             = 0;
/* Переменная для временного хранения значения "сырой" температуры с дачтика */
unsigned int  temperature         = 0;

/* Определение кнопок */
#define MODE_BUTTON     PD6
#define PLUS_BUTTON     PD7
#define MINUS_BUTTON    PB0

/* Определение времени задержки (мс) для устранения дребезга контактов */
#define DEB_INT		    150

/* Определение режимов работы и установок */
/* Режимы работы */
#define WRK_MODE        1
#define SET_MODE        2
#define CAL_MODE        3
/* Количество режимов */
#define MODE_CNT	    3
/* Настройки */
#define SET_TMP         0
#define SET_PWR         1
#define SET_VTG         2
#define SET_TIM         3
#define SET_BUZ	        4
#define SET_LUM		    5
#define SET_CTR	        6
/* Общее количество настроек и количество доступных настроек */
#define SET_EASTER_EGG	True
uint8_t enable_eegg	  =	False;
#define SET_CNT_MAX		7
uint8_t SET_CNT       = 3;

/* Определение границ и шага для настраиваемых значений */
/* Границы и шаг максимальной температуры */
#define temp_min        20
#define temp_max        90
#define temp_delta      5
/* Границы и шаг максимальной потребляемой мощности */
#define power_min       100
#define power_max       9900
#define power_delta     50
/* Границы и шаг значения входного сетевого напряжения */
#define voltage_min     180
#define voltage_max     250
#define voltage_delta   5
#define voltage_default 220
/* Границы и шаг таймера отключения нагрузки */
#define timer_min       0
#define timer_max       7200 // значение в секундах = 120 минут
#define timer_delta     60	 // значение в секундах = 1 минута
#define timer_default   0
/* Границы и шаг значения яркости дисплея */
#define lcd_light_min   0
#define lcd_light_max   255
#define lcd_light_delta 5
/* Границы и шаг значения контрастности дисплея */
#define lcd_contr_min   0
#define lcd_contr_max   255
#define lcd_contr_delta 5
#define lcd_contr_def   100

/* Объявление глобальных переменных */
/* Переменные запуска и режимов работы */
uint8_t  mode         = 0;
uint8_t  option       = 0;
uint8_t  launch       = True;
/* Переменные, хранящие значения настроек */
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
/* Определение первого запуска устройства */
uint8_t EEMEM  ee_first_run    = False;

/* Определение регистров таймеров для вывода ШИМ */
#define LIGHTNESS       OCR1A
#define CONTRAST        OCR1B

/* Определение регистра настроек для Buzzer */
#define BUZZER	        TCCR2

/* Определение каналов управления нагрузкой */
#define LOAD_PORT		PINC
#define CH1				PC2
#define CH2				PC1

/* Переменные для отсчета времени */
/* Счетчик прерываний таймера Т0 */
uint16_t timer_counter = 0;
/* Переменные таймера обратного отсчета */
uint8_t timer_enable  = False;
uint8_t timer_out     = False;
uint16_t timer_secs   = 0;

/* Переменные модуля выдачи звукового оповещения */
#define BUZZ_SIG_CNT    5
uint8_t BUZZED_TIMES  = 0;
uint8_t BUZZ_PART     = 0;
uint8_t BUZZ_CFG      = 0;

/* Значения температур каналов и общей мощности */
uint8_t CH1_temp      = 0;
uint8_t CH2_temp	  = 0;
uint16_t cur_power    = 0;

/* Флаг превышения потребляемой мощности */
uint8_t overpower	  = False;

/* Переменные, необходимые для работы АЦП */
unsigned long adc_value   = 0;
uint16_t      adc_counter = 0;
uint16_t      adc_noise   = 0;

void eeprom_load()
{
	/* Процедура загрузки настроек из EEPROM */
	/* Определение первого запуска */
	uint8_t first_run = eeprom_read_byte(&ee_first_run);
	if(first_run != False)
		eeprom_write_byte(&ee_first_run, False);
	/* Загрузка настроек из EEPROM */
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
}
void eeprom_save()
{
	/* Процедура сохранения настроек в EEPROM */
	eeprom_write_byte(&ee_set_max_tmp, set_max_tmp);
	eeprom_write_word(&ee_set_max_pwr, set_max_pwr);
	eeprom_write_byte(&ee_set_vtg, set_vtg);
	eeprom_write_word(&ee_set_timer, set_timer);
	eeprom_write_byte(&ee_set_light, set_light);
	eeprom_write_byte(&ee_set_contrast, set_contrast);
	eeprom_write_byte(&ee_set_buzzer, set_buzzer);	
}
void symbols_load()
{
	/* Процедура загрузки графических символов в дисплей */
	/* Символы для загрузки в дисплей */
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
	
	/* Загрузка символов в память дисплея */
	lcd_load(degree, 0);		// загрузка символа градуса по Цельсию
	lcd_load(thermometr, 1);	// загрузка символа термометра
	lcd_load(flash, 2);			// загрузка символа молнии
	lcd_load(timer, 3);			// загрузка символа таймера
	lcd_load(plug, 4);			// загрузка символа вилки
	lcd_load(light, 5);			// загрузка символа лампочки
	lcd_load(contrast, 6);		// загрузка символа контрастности
	lcd_load(sound, 7);			// загрузка символа динамика
}
void load_control(uint8_t channel, uint8_t state)
{
	/* Процедура упраления нагрузкой */
	switch (channel)
	{
		case 1:
			if(state)
				ENABLE(PORTC, CH1);
			else
				DISABLE(PORTC, CH1);		
			break;
		case 2:
			if(state)
				ENABLE(PORTC, CH2);
			else
				DISABLE(PORTC, CH2);		
			break;
		default:
			if(state)
			{
				ENABLE(PORTC, CH1);
				ENABLE(PORTC, CH2);				
			}
			else
			{
				DISABLE(PORTC, CH1);
				DISABLE(PORTC, CH2);
			}
			break;
	}
}
void startup()
{
	/* Начальная процедура настройки и запуска */
	/* Карта портов */
    /*
        PORT B:
               PB0 – MINUS BUTTON      (MODE, IN, PULLUP);
               PB1 – LIGHT	           (OC1A, OUT, 1);
               PB2 – CONTRAST          (OC1B, OUT, 1);
               PB3 – BUZZER            (OC2, OUT, 0);
               PB4 – N/A               (N/A, OUT, 0);
               PB5 – N/A               (N/A, OUT, 0);
               PB6 – OSC PIN           (8 MHz, IN, NO PULLUP);
               PB7 – OSC PIN           (8 MHz, IN, NO PULLUP);
        PORT C:
               PC0 – N/A               (N/A, OUT, 0);
               PC1 – CHANNEL 2 CONTROL (CH2, OUT, 0);
               PC2 – CHANNEL 1 CONTROL (CH1, OUT, 0);
               PC3 – TEMP MEASUREMENT  (1-WIRE, OUT, 0);
               PC4 – EASTER EGG        (EE, IN, PULLUP);
               PC5 – POWER MEASUREMENT (ADC5, IN, NO PULLUP);
               PC6 – RESET PIN         (RESET, IN, NO PULLUP);
               PC7 – N/A               (N/A, OUT, 0);
        PORT D:
			   PD0 – LCD PIN           (D7, OUT, 0);
			   PD1 – LCD PIN           (D6, OUT, 0);
			   PD2 – LCD PIN           (D5, OUT, 0);
			   PD3 – LCD PIN           (D4, OUT, 0);
			   PD4 – LCD PIN           (E, OUT, 0);
			   PD5 – LCD PIN           (RS, OUT, 0);
			   PD6 – MODE BUTTON       (MODE, IN, PULLUP);
			   PD7 – PLUS BUTTON       (MODE, IN, PULLUP);   
    */

    /* Инициализация портов */
    DDRB  = 0x3E;
	PORTB = 0x07;
    DDRC  = 0x8F;
    PORTC = 0x10;
    DDRD  = 0x3F;
    PORTD = 0xC0;

	/* Настройка таймеров */
	TIMSK|=(1<<TOIE0);
	// Разрашение прерывания по переполнению таймера T0
	TCCR0|=(0<<CS02)|(0<<CS01)|(1<<CS00);
	// Prescaller: N = 1; f = 8 MHz 
	TCCR1A|=(1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(1<<WGM10);
	// PB1 (OC1A = LIGHT) и PB2 (OC1B = CONTRAST) выдают Phase-Correct PWM, 8-bit, TOP = 0x00FF
	TCCR1B|=(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10);
	// Prescaller: N = 1, f = 8 MHz / (2 * N * TOP) = 15.686275 kHz
	TCCR2|=(1<<COM21)|(0<<COM20)|(1<<WGM21)|(1<<WGM20)|(0<<CS22)|(1<<CS21)|(1<<CS20);
	// PB3 (OC2 = BUZZER) выдает Fast PWM, TOP = 0xFF, Prescaller: N = 32, f = 8 MHz / (N * 256) = 976.5625 Hz
	OCR2=0x7F;
	/*		f = 976.5625 Hz = 0.9765625 kHz
			T = 1/f = 1/0.9765625 = 1.024 s
			S = T/t = TOP/OCR2 = 255/127 = 2.007874
			t = T/S = 1.024/2.007874 = 0.509992 s
	*/
	
	/* Настройка АЦП */
	ADMUX|=(1<<REFS1)|(1<<REFS0)|(0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(1<<MUX0);
	// ИОН: внутренний, 2.56V; вход АЦП: ADC5
	ADCSRA|=(0<<ADSC)|(0<<ADFR)|(1<<ADIE)|(0<<ADPS2)|(0<<ADPS1)|(0<<ADPS0);
	// Измерение по запросу, прерывание по окончании преобразования, делитель 2: f ADC = 8 MHz / 2 = 4 MHz	
	
	/* Блок для отладки в PROTEUS */
		TCCR1B|=(0<<WGM13)|(0<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10);
		// Prescaller: N = 1024, f = 8 MHz / (2 * N * TOP) = 15.318627 Hz
		TCCR2|=(1<<COM21)|(0<<COM20)|(1<<WGM21)|(1<<WGM20)|(1<<CS22)|(1<<CS21)|(1<<CS20);
		// Prescaller: N = 1024, f = 8 MHz / (1024 * 256) = 30.517578 Hz
		SET_CNT = SET_CNT_MAX;
		// Включено ВСЕ меню
	/* Блок для отладки в PROTEUS */
	
	/* Сохранение настроек таймера OC2 для Buzzer */
	BUZZ_CFG = BUZZER;
	
	/* Выключение Buzzer */
	BUZZER = 0;

	/* Проверка аппаратной возможности использовать Easter Egg */
	if(CHECKBIT(PINC, PC4) == 0)
		enable_eegg = True;
	else
		enable_eegg = False;

	/* Загрузка параметров из EEPROM */
	eeprom_load();		

    /* Карта LCD-дисплея */
    /*
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01| *| T| 1| =| 9| 5| °| C|  | P| =| 9| 9| 9| 9| W|
        |02| *| T| 2| =| 9| 5| °| C|  | T| =| 0| 1| :| 4| 7|
        
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01|  |  |  |  | S| E| T| T| I| N| G| S|  |  |  |  |
        |02|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |               
    */

	/* Инициализация шины 1-Wire */
	OWI_Init(BUS);

    /* Инициализация дисплея */
    lcd_init();		// инициализация дисплея
    lcd_clrscr();	// очистка дисплея	
	
	/* Вывод приглашения для выполнения калибровки */
	lcd_goto(1, 0);
	lcd_prints("\tTo calibrate");
	lcd_goto(2, 0);
	lcd_prints("\tPress \"MODE\"");	
	/* Глобальное разрешение прерываний:
			– команда asm("sei") вынесена в процедуру calibrate(),
			  т.к. так не "глючит" память.
	*/
}
void settings()
{
    /* Процедура настройки параметров */
    lcd_goto(1, 0);
    lcd_prints("\t\tSETTINGS");  
	lcd_goto(2, 0);
	switch(option)
    {
	    case SET_TMP:
			// Настройка максимально-допустимой температуры
			lcd_prints("\t ");
			lcd_putc(1);
			lcd_prints("max: ");
			lcd_numTOstr(set_max_tmp, 2);
			lcd_putc(0);
			lcd_prints("C");
			break;
	    case SET_PWR:
			// Настройка максимально-допустимой мощности
			lcd_prints("\t");
			lcd_putc(2);
			lcd_prints("max: ");
			lcd_numTOstr(set_max_pwr, 4);
			lcd_prints(" W");
			break;
	    case SET_VTG:
			// Настройка напряжения сети
			lcd_prints("\t ");
			lcd_putc(4);
			lcd_prints("in: ");
			lcd_numTOstr(set_vtg, 3);
			lcd_prints(" V");
			break;
	    case SET_TIM:
			// Настройка таймера отключения нагревателя (поддержание заданной температуры)
			lcd_prints("\t\t");
			lcd_putc(3);
			lcd_prints(": ");
			lcd_numTOstr(set_timer/3600, 2);
			lcd_prints(":");
			lcd_numTOstr((set_timer/60)%60, 2);
			break;
	    case SET_BUZ:
			// Настройка звукового оповещения
			lcd_prints("\t\t ");
			lcd_putc(7);
			lcd_prints(": ");
			if(set_buzzer)
				lcd_prints("ON ");
			else
				lcd_prints("OFF");
			break;
		case SET_LUM:
			// Настройка яркости дисплея
			lcd_prints("\t\t ");
			lcd_putc(5);
			lcd_prints(": ");
			lcd_numTOstr(set_light, 3);
			break;
		case SET_CTR:
			// Настройка контрастности дисплея
			lcd_prints("\t\t ");
			lcd_putc(6);
			lcd_prints(": ");
			lcd_numTOstr(set_contrast, 3);
			break;						
    }	
}
void buzz(uint8_t state)
{
	/* Процедура включения/отключения аккустического сигнала (Buzzer) */
	if(state)
		BUZZER = BUZZ_CFG;
	else
		BUZZER = 0;
}
void timer_reset()
{
	/* Процедура сброса таймера */
	timer_enable = False;
	timer_out = False;
	timer_secs = 0;	
}

void ds18b20_search()
{
    /* Процедура поиска датчиков температуры */
	/* Если флаг сброшен - выполнить поиск 1-Wire устройств.
       Если кол-во заданных устройсв совпадает с кол-вом найденных -
	   устанавливаем флаг, чтобы функция поиска больше не выполнялась. */
    if (searchFlag == SEARCH_SENSORS)
	{
		dev_num = 0;
		crcFlag = OWI_SearchDevices(allDevices, MAX_DEVICES, BUS, &dev_num);
		/* TEST BLOCK */
			lcd_goto(1, 0);
			lcd_prints("N = ");
			lcd_itostr(dev_num);
		/* TEST BLOCK */
		if((dev_num == MAX_DEVICES) && (crcFlag != SEARCH_CRC_ERROR))
			searchFlag = SENSORS_FOUND;
    }	
}
unsigned char ds18b20_read_temp(unsigned char bus, unsigned char * id, unsigned int* temperature)
{
    /* Процедура чтения температуры с датчика */
	unsigned char scratchpad[9];
    unsigned char i;
    /* Подаем сигнал сброса.
	   Подаем команду адрессации 1-Wire устройства на шине.
       Подаем команду запуска преобразования. */
    OWI_DetectPresence(bus);
    OWI_MatchRom(id, bus);
    OWI_SendByte(DS18B20_CONVERT_T ,bus);
    /* Ждем завершения преобразования */ 
    while(!OWI_ReadBit(bus));
    /* Подаем сигнал сброса.
	   Подаем команду для адрессации 1-Wire устройства на шине.
       Подаем команду чтения внутренней памяти.
       Считываем внутреннюю память датчика в массив. */	
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
void ds18b20_convert_temp(unsigned int temperature)
{
	/* Процедура конвертации температуры */
	unsigned char tmp = 0;
	/* Если температура отрицательная, выполняем преобразование */ 
	if((temperature & 0x8000) != 0)
		temperature = ~temperature + 1;
	/* Целая часть температуры */   
	tmp = (unsigned char)(temperature>>4);
	/* TEST BLOCK */
		lcd_itostr(tmp);
	/* TEST BLOCK */
	/* Дробная часть температуры */  
	tmp = (unsigned char)(temperature&15);
	tmp = (tmp>>1) + (tmp>>3);
	/* TEST BLOCK */
		lcd_prints(".");
		lcd_itostr(tmp);
	/* TEST BLOCK */	
}
void ds18b20_show_temp(uint8_t dev_id)
{
	/* Процедура вывода температуры */
	crcFlag = ds18b20_read_temp(BUS, allDevices[dev_id].id, &temperature);
	if(crcFlag != READ_CRC_ERROR)
		ds18b20_convert_temp(temperature);
	else
	{
		lcd_prints("N/A");
		searchFlag = SEARCH_SENSORS;
	}	
}
uint16_t get_adc_value(uint16_t count)
{
	/* Процедура получения значения с АЦП */
	adc_value   = 0;
	adc_counter = 0;
	unsigned long adc_values_accumulator = 0;
	uint16_t bar_count = count/NUMBER_OF_BAR_ELEMENTS;
	/* Сбор данных с АЦП и вывод шкалы прогресса (в случае режима калибровки) */
	for(uint16_t i=0; i<count; i++)
	{
		ADCSRA|=(1<<ADSC);
		adc_values_accumulator += adc_value;
		if(mode == CAL_MODE)
			lcd_drawbar(i/bar_count);
	}
	/* Возвращение значения */
	return adc_values_accumulator / adc_counter;
}
uint16_t get_power_value()
{
	/* Таблица значений "величина-ток" для датчика SCT-013-000:
	      50 mV => 100 A
	       2 mV => 1 A
		   1 mV => 0.5 A
	     0.1 mV => 0.05 A
		  U(mV) =  I(A) * 0.5
	  
	  Данные АЦП:
		BITs = 1024
		Vref = 2.56 V
		 ADC = ((Vin * 1024) / Vref)
		  dV = 2.56 / 1024 = 2.5 mV / bit => 0.0025 V / bit
	*/
	
	/* Получаем значение напряжения с АЦП и вычитаем из него шум */
	uint16_t power_value = get_adc_value(10) - adc_noise;
	/* Переводим полученное значение в напряжение датчика */
	//power_value = (power_value * 2.56) / 1024;
	/* Переводим полученное значение в ток потребления */
	/* TEST BLOCK */
		// здесь нужно написать код для преобразования напряжения в ток
	/* TEST BLOCK */
	/* Вычисляем значение потребляемой мощности */
	//power_value = power_value * set_vtg;
	/* Возвращаем полученное значение */
	return power_value;
}
void values_refresh()
{
	/* Процедура обновления значений параметров, при их изменении.
	   Вдобавок, так же выключает нагрузки при достижении температуры канала/превышении мощности. */
	/* Получение значения потребляемой мощности */
//	cur_power = get_power_value();
	/* Получение температур каналов */
// 	CH1_temp =
// 	CH2_temp =
	/* Аварийное отключение нагрузки при превышении потребляемой мощности */
	if(cur_power >= set_max_pwr)
	{
		overpower = True;
		timer_reset();		
		load_control(0, Off);
		buzz(On);
	}
	else
	{
		overpower = False;
		if(!timer_out)
			buzz(Off);
	}
	/* Управление каналами нагрузки в зависимости от температуры */
	if((!overpower) && (mode != 0) && (mode != CAL_MODE))
	{
		/* Если таймер выключен */
		if((set_timer == 0) || (timer_enable))
		{
			if(CH1_temp < set_max_tmp)
				load_control(1, On);
			else
				load_control(1, Off);
			if(CH2_temp < set_max_tmp)
				load_control(2, On);
			else
				load_control(2, Off);
		}
	}
	else
		load_control(0, Off);
	/* Обновление значений яркости и контраста для таймера */
	if(LIGHTNESS != set_light)
		LIGHTNESS = set_light;
	if(CONTRAST != set_contrast)
		CONTRAST = set_contrast;	
}
void temp_out(uint8_t channel, uint8_t temp_value, uint8_t status)
{
    /* Процедура вывода значения температуры на дисплей */
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
    /* Процедура вывода значения мощности на дисплей */
    lcd_goto(1, 9);
    lcd_putc(2);
	lcd_prints("=");
	lcd_numTOstr(power_value, 4);
    lcd_prints("W");
}
void time_out(uint16_t set_timer, uint16_t timer_secs)
{
    /* Процедура вывода оставшегося времени на дисплей */
	if(timer_enable && !timer_out)
	{
		uint16_t time  = set_timer-timer_secs;
		uint16_t tmp   = (time/3600)*3600;
		uint8_t  first = 0;
		uint8_t second = 0;
		if(time < 3600)
		{
			first  = (time-tmp)/60;		  // минуты
			second = time-tmp-(first*60); // секунды
		}
		else
		{
			first  = time/3600;		      // часы
			second = (time-tmp)/60;	      // минуты
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
	/* Процедура вывода данных на основной экран */
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
	/* Процедура калибровки АЦП для устранения шумов */
	/* Включение АЦП */
	ADCSRA|=(1<<ADEN);
	/* Запуск первого преобразования АЦП */
	ADCSRA|=(1<<ADSC);
	/* Вывод информации */
	lcd_clrscr();
	lcd_goto(1, 0);
	lcd_prints(" CALIBRATING...");
	lcd_goto(2, 0);
	/* Глобальное разрешение прерываний */
	sei();
	/* Вычисление значения шума */	
	adc_noise = get_adc_value(1000);	// значение шума
	/* TEST BLOCK */
		lcd_clrscr();
		lcd_goto(1, 0);
		lcd_prints("ADC Noise = ");
		lcd_itostr(adc_noise);
// 		char ch_array[10];
// 		lcd_goto(2, 0);
// 		lcd_prints("U Noise = ");
// 		dtostrf(adc_noise*0.0025, 1, 4, ch_array);
// 		lcd_prints(ch_array);
		_delay_ms(2000);
	/* TEST BLOCK */
	/* Глобальный запрет прерываний */
	cli();
	/* Загрузка символов в дисплей */
	symbols_load();
	/* Глобальное разрешение прерываний */
	sei();
	/* Переход в рабочий режим */
	lcd_clrscr();
	mode = WRK_MODE;
}
void buttons_check()
{
    /* Процедура проверки нажатия кнопок */
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
					mode = WRK_MODE;
                }
            }
            else
            {
                if(mode < MODE_CNT-1)
                    mode++;
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
		if(CHECKBIT(PINC, PC4) != 0)
		{
			if((mode == WRK_MODE) && (CHECKBIT(PIND, PLUS_BUTTON) == 0) && (CHECKBIT(PINB, MINUS_BUTTON) == 0))
				SET_CNT = SET_CNT_MAX;
		}
	}
}

ISR(TIMER0_OVF_vect)
{
	/* Вектор прерывания по переполнению таймера Т0 */
	timer_counter++;
	/* Действия по прошествии одной секунды */
	if(timer_counter>=31250)
	{
		/* Обработчик для таймера обратного отсчета */
		if(timer_enable)
		{
			/* Увеличение счетчика секунд */
			timer_secs++;
			if(timer_secs >= set_timer)
			{
				/* Действия при завершении таймера обратного отсчета */
				/* Отключаем таймер обратного отсчета, обнуляя счетчики минут и секунд */
				timer_reset();
				/* Отключаем все нагрузки */
				load_control(0, Off);
				/* Помечаем таймер, как отработавший */
				timer_out = True;	
			}
		}
		/* Обработчик для выдачи звукового сигнала */
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
		/* Обнуление счетчика прерываний таймера Т0 */
		timer_counter = 0;		
	}
}
ISR(ADC_vect)
{
	adc_value = ADC;
	adc_counter++;
}

int main(void)
{
    startup();
	
	/* TEST BLOCK */
		set_max_tmp = 55;
		CH1_temp = 50;
		CH2_temp = 30;
	/* TEST BLOCK */
	
	/* TEST BLOCK - GET TEMP */	
		lcd_clrscr();
	/* TEST BLOCK - GET TEMP */	
	
	while(1)
    {	
		/* TEST BLOCK - GET TEMP */
			ds18b20_search();
			lcd_goto(2, 0);
			lcd_prints("T1=");
			ds18b20_show_temp(0);
			lcd_goto(2, 8);
			lcd_prints("T2=");		
			ds18b20_show_temp(1);	
		/* TEST BLOCK - GET TEMP */
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
		values_refresh();
    }
    return 0;
}