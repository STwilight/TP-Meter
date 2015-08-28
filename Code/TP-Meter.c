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

/* ќпределение рабочей частоты ћ  */
#define F_CPU 8000000

/* ѕодключение библиотек */
#include <avr/io.h>
#include <util/delay.h>
#include "hd44780.h"

/* ќпределение спец. значений */
#define True  1
#define False 0

/* ќпределение кнопок */
#define MODE_BUTTON  PC3
#define PLUS_BUTTON  PC4
#define MINUS_BUTTON PC5

/* ќпределение времени (в мс) дл€ устранени€ дребезга контактов */
#define debounce_interval 500

/* ќпределение режимов работы и установок */
/* –ежимы работы */
#define WRK_MODE      1
#define SET_MODE      2
#define CAL_MODE      3
/* Ќастройки */
#define SET_TMP       0
#define SET_PWR       1
#define SET_VTG       2
#define SET_TIM       3
/*  оличество настроек */
#define SET_CNT       4

/* ќпределение границ и шага дл€ настраиваемых значений */
#define temp_min      20
#define temp_max      90
#define temp_delta    10
#define power_min     100
#define power_max     9900
#define power_delta   100
#define voltage_min   180
#define voltage_max   240
#define voltage_delta 10
#define time_min      10
#define time_max      120 
#define time_delta    10

/* ќбъ€вление глобальных переменных */
uint8_t mode   = 0;
uint8_t option = 0;
uint8_t launch = True;
uint8_t  max_tmp = 0;
uint16_t max_pwr = 0;
uint8_t net_vtg = 0;
uint8_t timer_value = 0;
uint8_t timer_current = 0;

void startup()
{
	/*  арта портов */
    /*
        PORT B:
               PB0 Ц LCD PIN           (RS, OUT, 0);
               PB1 Ц LCD PIN           (E, OUT, 0);
               PB2 Ц LCD PIN           (D4, OUT, 0);
               PB3 Ц LCD PIN           (D5, OUT, 0);
               PB4 Ц LCD PIN           (D6, OUT, 0);
               PB5 Ц LCD PIN           (D7, OUT, 0);
               PB6 Ц OSC PIN           (8 MHz, IN, NO PULLUP);
               PB7 Ц OSC PIN           (8 MHz, IN, NO PULLUP);
        PORT C:
               PC0 Ц POWER MEASUREMENT (ADC0, IN, NO PULLUP);
               PC1 Ц CHANNEL 1 CONTROL (CH1, OUT, 0);
               PC2 Ц CHANNEL 2 CONTROL (CH2, OUT, 0);
               PC3 Ц MODE BUTTON       (MODE, IN, PULLUP);
               PC4 Ц PLUS BUTTON       (MODE, IN, PULLUP);
               PC5 Ц MINUS BUTTON      (MODE, IN, PULLUP);
               PC6 Ц RESET PIN         (RESET, IN, NO PULLUP);
               PC7 Ц N/A;              (N/A, OUT, 0);
        PORT D:
               PD0 Ц TEMP MEASUREMENT  (1-WIRE, OUT, 0);
               PD1 Ц N/A;              (N/A, OUT, 0);
               PD2 Ц N/A;              (N/A, OUT, 0);
               PD3 Ц BUZZER            (OC2B, OUT, 0);
               PD4 Ц N/A;              (N/A, OUT, 0);
               PD5 Ц BACKLIGHT         (OC0B, OUT, 1);
               PD6 Ц CONTRAST          (OC0A, OUT, 1);
               PD7 Ц N/A;              (N/A, OUT, 0);
    */

    /* »нициализаци€ портов */
    DDRB  = 0x3F;
	PORTB = 0x00;
    DDRC  = 0x86;
    PORTC = 0x38;
    DDRD  = 0xFF;
    PORTD = 0x60;
    
	/* —имволы дл€ загрузки в дисплей */
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
    uint8_t timer[8]=
    {
        0b00000000,
        0b00001110,
        0b00010011,
        0b00010101,
        0b00010101,
        0b00001110,
        0b00000000,
        0b00000000
    };          

    /*  арта LCD-диспле€ */
    /*
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01| *| T| 1| =| 9| 5| ∞| C|  | P| =| 9| 9| 9| 9| W|
        |02| *| T| 2| =| 9| 5| ∞| C|  |  |  |  |  |  |  |  |
        
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01| *| T| =| 9| 5| ∞| C|  | *| P| =| 9| 9| 0| 0| W|
        |02| *| U| =| 2| 3| 0| V|  | *| t| =| 2| 5| m| i| n| 
        
           |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|
        |01|  |  |  |  | S| E| T| T| I| N| G| S|  |  |  |  |
        |02|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |               
    */

    /* »нициализаци€ диспле€ */
    lcd_init();				// инициализаци€ диспле€
    lcd_clrscr();           // очистка диспле€
	lcd_load(degree, 0);	// загрузка символа градуса по ÷ельсию
    lcd_load(timer, 1);     // загрузка символа таймера
    
    /* ¬ывод приглашени€ дл€ выполнени€ калибровки */
    lcd_goto(1, 0);
    lcd_prints("\tTo calibrate");
    lcd_goto(2, 0);
    lcd_prints("\tPress \"MODE\"");
}
void calibrating()
{
    /* ѕроцедура калибровки ј÷ѕ дл€ устранени€ шумов */
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
    mode = WRK_MODE;
}
void setup()
{
    /* ѕроцедура дл€ установки значений */
}
void temp_out(uint8_t channel, uint8_t temp_value, uint8_t status)
{
    /* ћетод дл€ вывода значени€ температуры на дисплей */
    lcd_goto(channel, 0);
    if(status)        
        lcd_prints("*");
    else
        lcd_prints(" ");
    lcd_prints("T");
    lcd_itostr(channel);
    lcd_prints("=");
    lcd_numTOstr(temp_value, 2);
    lcd_putc(0);
    lcd_prints("C");
}
void power_out(uint16_t power_value)
{
    /* ћетод дл€ вывода значени€ мощности на дисплей */
    lcd_goto(1, 9);
    lcd_prints("P=");
    lcd_numTOstr(power_value, 4);
    lcd_prints("W");
}
void time_out(uint8_t timer_value, uint8_t timer_current)
{
    /* ћетод дл€ вывода оставшегос€ времени на дисплей */
    // врем€, переданное в минутах, переводитс€ в часы
    uint8_t time = timer_value - timer_current;
    uint8_t hours = time / 60;
    uint8_t mins = time % 60;
    lcd_goto(2, 9);
    //lcd_prints("Tx=");
    lcd_putc(1);
    lcd_prints(" ");
    lcd_numTOstr(hours, 2);  
    lcd_prints(":");
    lcd_numTOstr(mins, 2);
}
void measure(uint8_t temperature, uint16_t power)
{
    /* ѕроцедура измерени€ входных параметров */
}
void buttons_check()
{
    if(CHECKBIT(PINC, MODE_BUTTON) == 0)
    {
        _delay_ms(debounce_interval);
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
                    option = SET_TMP;
                    mode = WRK_MODE;
                }
            }
            else
            {
                if(mode < SET_MODE)
                    mode++;
                else
                    mode = WRK_MODE;            
            }
        }
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
                temp_out(1, 27, False);
                temp_out(2, 32, True);
                power_out(9900);
                time_out(85, 13); // таймер = 85 мин = 1:25, прошло = 0:13, осталось = 1:12
                break;
            case SET_MODE:
                switch(option)
                {
                    case SET_TMP:
                        // Ќастройка максимально-допустимой температуры
                        lcd_goto(1, 0);
                        lcd_prints("\t\tSETTINGS");
                        lcd_goto(2, 0);
                        lcd_prints("\tTmax = ");
                        lcd_numTOstr(max_tmp, 2);
                        lcd_prints(" ");
                        lcd_putc(0);
                        lcd_prints("C");             
                        break;
                    case SET_PWR:
                        // Ќастройка максимально-допустимой мощности
                        lcd_goto(1, 0);
                        lcd_prints("\t\tSETTINGS");
                        lcd_goto(2, 0);
                        lcd_prints("\tPmax = ");
                        lcd_numTOstr(max_pwr, 4);
                        lcd_prints(" ");
                        lcd_prints("W");                                       
                        break;                    
                    case SET_VTG:
                        // Ќастройка напр€жени€ сети
                        lcd_goto(1, 0);
                        lcd_prints("\t\tSETTINGS");
                        lcd_goto(2, 0);
                        lcd_prints("\tUin = ");
                        lcd_numTOstr(net_vtg, 3);
                        lcd_prints(" ");
                        lcd_prints("V");                                         
                        break;                    
                    case SET_TIM:
                        // Ќастройка таймера отключени€ нагревател€ (поддержание заданной температуры)
                        lcd_goto(1, 0);
                        lcd_goto(1, 0);
                        lcd_prints("\t\tSETTINGS");
                        lcd_goto(2, 0);
                        lcd_prints(" TIMER = ");
                        lcd_numTOstr(timer_value, 3);
                        lcd_prints(" ");
                        lcd_prints("min");
                        break;                    
                }
                break;
            case CAL_MODE:
                calibrating();
                break;            
        }
    }
    return 0;
}