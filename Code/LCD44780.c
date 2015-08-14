/*
 * LCD44780.c
 * 
 * Created: 14.08.2015
 * CPU: ATMega48V
 * Frequency: 8 MHz
 * Author: Symrak
 *
 */ 

#define F_CPU 8000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "hd44780.h"

/*
long val;
int adc_counter, state=0;
double fval=0;
char mval[10]="0";
char nval[10]="0";
int ndights=10;
	
ISR(ADC_vect)
{
	val=val+ADC;
	adc_counter++;
}
// Прерывание по окончанию преобразования АЦП
*/

void startup()
{
	DDRB=0xFF;				// порт B на выход
	PORTB=0x00;				// лог. 0
	/*
	DDRC=0x00; 	 // порт C на вход
	PORTC=0xFF;	 // подтягивающие резисторы включены
	DDRD=0x80;   // PD0..PD6 на вход, PD7 на выход
	PORTD=0x7F;	 // PD0..PD6 подтягивающие резисторы включены, PD7 – лог. 0
	*/
	
	/* знак градуса для загрузки в дисплей */
	uint8_t degree[8]=
	{  
		0x0E, // 0 1 1 1 0
		0x0A, // 0 1 0 1 0
		0x0E, // 0 1 1 1 0
		0x00, // 0 0 0 0 0
		0x00, // 0 0 0 0 0
		0x00, // 0 0 0 0 0
		0x00, // 0 0 0 0 0
		0x00  // 0 0 0 0 0
	};	
	
	lcd_init();				// инициализация дисплея
	lcd_load(degree,0);		// загрузка символа градуса по Цельсию
	lcd_clrscr();			// очистка дисплея
	lcd_goto(1,0);			// переход на строчку 1
	lcd_prints("TP-Meter");	// вывод надписи с табулированием (/t)
	lcd_goto(2,0);			// переход на строчку 2
		for(int i=0;i<NUMBER_OF_BAR_ELEMENTS;i++)
		{
			lcd_drawbar(i);
			_delay_ms(5);
		}	// прогресс-бар
	lcd_clrscr();			// очистка экрана
	lcd_goto(1,0);			// переход на строчку 1
	lcd_prints("t=27.0C");	// вывод температуры
	lcd_goto(1,7);			// переход в поле вывода символа температуры
	lcd_putc(0);			// вывод символа градуса Цельсия
	lcd_goto(2,0);			// переход на строчку 1
	lcd_prints("P=50.1kW");	// вывод мощности
	// lcd_prints("P=999.9W");	// вывод мощности (=< 999.90 ---> W)
	// lcd_prints("P=99.9kW");	// вывод мощности (>= 1000.0 ---> kW)
	
	
	/* настройка АЦП */
	/*
	ADCSRA|=(1<<ADEN)// разрешение АЦП
	|(1<<ADSC) // запуск преобразования
	|(1<<ADFR) // непрерывный режим работы АЦП
	|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0) // предделитель на 64 (частота АЦП 125kHz)
	|(1<<ADIE); // разрешение прерываний
	
	ADMUX|=(1<<REFS1)|(1<<REFS0) // внутренний ИОН 2,56V
	|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0); // выбор входа ADC0
	*/
	
	/* настройка АЦП */
	
	/*
	_delay_ms(50);	// задержка перед началом использования АЦП
	sei();			//глобальное разрешение прерываний				
	*/
}

int main(void)
{
    startup();	
	while(1)
    {
	
		/*
		char xval[10]="0";
		itoa(85,xval,2); // задание значения 85 для перевода в двоичное
		lcd_goto(1,0);
		lcd_prints(xval); // вывод двоичного исходного
		lcd_goto(1,strlen(xval)+1);
		lcd_itostr(strlen(xval)); // вывод длины двоичного исходного
		
		for(int i=0;i<(ndights-strlen(xval));i++) // заполнение нулями слева
			nval[i]=48;	

		for(int i=(ndights-strlen(xval));i<=(ndights);i++) // заполнение символами массива
			nval[i]=xval[i-(ndights-strlen(xval))];	
		
		lcd_goto(2,0);
		lcd_prints(nval);
		_delay_ms(500);	
		*/
		
		/*
		if(adc_counter>300) // вычисляем среднее значение АЦП
		{
			switch (state)
			{
				case 0: // выводим номер уровеня и его двоичное представление
					lcd_goto(1,0);
					lcd_prints("LVL: ");
					lcd_goto(2,0);
					lcd_prints("VAL: ");
					lcd_goto(1,5);
					lcd_itostr(val/adc_counter);
					lcd_goto(2,5);
					itoa(val/adc_counter,mval,2);
					// преобразование dec в bin
					for(int i=0;i<(ndights-strlen(mval));i++) // заполнение нулями слева
						nval[i]=48;
					for(int i=(ndights-strlen(mval));i<=(ndights);i++) // заполнение символами массива
						nval[i]=mval[i-(ndights-strlen(mval))];	 			
					//lcd_prints(mval);				
					lcd_prints(nval);
					// вывод с заполнением нулями
					break;
				case 1: // меряем U и dBV
					lcd_goto(1,0);
					lcd_prints("Uin: ");
					lcd_goto(1,5);
					fval=(val/adc_counter)*0.0025;
					// Umax = [1023*2.56 V]/1024 = 2.5575 V
					// K = 2.5575 V/1023 = 0.0025
					dtostrf(fval,1,4,mval);
					// преобоазование float в char
					// [float, мин. ширина вывода (с уч. "." и знака), зн. после ".", char]
					lcd_prints(mval);
					fval=20*log10(fval/1);
					// dBV: вычисление 20 десятичных логарифмов отношения напряжения к опорному в 1 V
					lcd_goto(2,0);
					lcd_prints("dBV: ");
					lcd_goto(2,5);
					dtostrf(fval,1,4,mval);
					// преобоазование float в char
					lcd_prints(mval);														
					break;
				case 2: // меряем U и dBu
					lcd_goto(1,0);
					lcd_prints("Uin: ");
					lcd_goto(1,5);
					fval=(val/adc_counter)*0.0025;
					// Umax = [1023*2.56 V]/1024 = 2.5575 V
					// K = 2.5575 V/1023 = 0.0025
					dtostrf(fval,1,4,mval);
					// преобоазование float в char
					// [float, мин. ширина вывода (с уч. "." и знака), зн. после ".", char]
					lcd_prints(mval);
					fval=20*log10(fval/0.775);
					// dBu: вычисление 20 десятичных логарифмов отношения напряжения к опорному в 0.775 V
					lcd_goto(2,0);
					lcd_prints("dBu: ");
					lcd_goto(2,5);
					dtostrf(fval,1,4,mval);
					// преобоазование float в char
					lcd_prints(mval);															
					break;				
				default: break;
			}
			if((PIND&(1<<PD0))==0) // если на PD0 логический 0 (кнопка нажата), то меняем state
			{
				_delay_ms(10); // программный антидребезг контактов
				PORTD=PORTD|0x80; // OR = PB7 подается лог. 1
				_delay_ms(250);
				PORTD=PORTD^0x80; // XOR = PB7 подается лог. 0
				state++;				
					if(state>2)
					state=0;
			}
			adc_counter=0;
			val=0;			
		}
		_delay_ms(100);
		// задержка перед выводом следующего значения
		*/
    }
	return 0;
}