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
// ���������� �� ��������� �������������� ���
*/

void startup()
{
	DDRB=0xFF;				// ���� B �� �����
	PORTB=0x00;				// ���. 0
	/*
	DDRC=0x00; 	 // ���� C �� ����
	PORTC=0xFF;	 // ������������� ��������� ��������
	DDRD=0x80;   // PD0..PD6 �� ����, PD7 �� �����
	PORTD=0x7F;	 // PD0..PD6 ������������� ��������� ��������, PD7 � ���. 0
	*/
	
	/* ���� ������� ��� �������� � ������� */
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
	
	lcd_init();				// ������������� �������
	lcd_load(degree,0);		// �������� ������� ������� �� �������
	lcd_clrscr();			// ������� �������
	lcd_goto(1,0);			// ������� �� ������� 1
	lcd_prints("TP-Meter");	// ����� ������� � �������������� (/t)
	lcd_goto(2,0);			// ������� �� ������� 2
		for(int i=0;i<NUMBER_OF_BAR_ELEMENTS;i++)
		{
			lcd_drawbar(i);
			_delay_ms(5);
		}	// ��������-���
	lcd_clrscr();			// ������� ������
	lcd_goto(1,0);			// ������� �� ������� 1
	lcd_prints("t=27.0C");	// ����� �����������
	lcd_goto(1,7);			// ������� � ���� ������ ������� �����������
	lcd_putc(0);			// ����� ������� ������� �������
	lcd_goto(2,0);			// ������� �� ������� 1
	lcd_prints("P=50.1kW");	// ����� ��������
	// lcd_prints("P=999.9W");	// ����� �������� (=< 999.90 ---> W)
	// lcd_prints("P=99.9kW");	// ����� �������� (>= 1000.0 ---> kW)
	
	
	/* ��������� ��� */
	/*
	ADCSRA|=(1<<ADEN)// ���������� ���
	|(1<<ADSC) // ������ ��������������
	|(1<<ADFR) // ����������� ����� ������ ���
	|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0) // ������������ �� 64 (������� ��� 125kHz)
	|(1<<ADIE); // ���������� ����������
	
	ADMUX|=(1<<REFS1)|(1<<REFS0) // ���������� ��� 2,56V
	|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0); // ����� ����� ADC0
	*/
	
	/* ��������� ��� */
	
	/*
	_delay_ms(50);	// �������� ����� ������� ������������� ���
	sei();			//���������� ���������� ����������				
	*/
}

int main(void)
{
    startup();	
	while(1)
    {
	
		/*
		char xval[10]="0";
		itoa(85,xval,2); // ������� �������� 85 ��� �������� � ��������
		lcd_goto(1,0);
		lcd_prints(xval); // ����� ��������� ���������
		lcd_goto(1,strlen(xval)+1);
		lcd_itostr(strlen(xval)); // ����� ����� ��������� ���������
		
		for(int i=0;i<(ndights-strlen(xval));i++) // ���������� ������ �����
			nval[i]=48;	

		for(int i=(ndights-strlen(xval));i<=(ndights);i++) // ���������� ��������� �������
			nval[i]=xval[i-(ndights-strlen(xval))];	
		
		lcd_goto(2,0);
		lcd_prints(nval);
		_delay_ms(500);	
		*/
		
		/*
		if(adc_counter>300) // ��������� ������� �������� ���
		{
			switch (state)
			{
				case 0: // ������� ����� ������� � ��� �������� �������������
					lcd_goto(1,0);
					lcd_prints("LVL: ");
					lcd_goto(2,0);
					lcd_prints("VAL: ");
					lcd_goto(1,5);
					lcd_itostr(val/adc_counter);
					lcd_goto(2,5);
					itoa(val/adc_counter,mval,2);
					// �������������� dec � bin
					for(int i=0;i<(ndights-strlen(mval));i++) // ���������� ������ �����
						nval[i]=48;
					for(int i=(ndights-strlen(mval));i<=(ndights);i++) // ���������� ��������� �������
						nval[i]=mval[i-(ndights-strlen(mval))];	 			
					//lcd_prints(mval);				
					lcd_prints(nval);
					// ����� � ����������� ������
					break;
				case 1: // ������ U � dBV
					lcd_goto(1,0);
					lcd_prints("Uin: ");
					lcd_goto(1,5);
					fval=(val/adc_counter)*0.0025;
					// Umax = [1023*2.56 V]/1024 = 2.5575 V
					// K = 2.5575 V/1023 = 0.0025
					dtostrf(fval,1,4,mval);
					// �������������� float � char
					// [float, ���. ������ ������ (� ��. "." � �����), ��. ����� ".", char]
					lcd_prints(mval);
					fval=20*log10(fval/1);
					// dBV: ���������� 20 ���������� ���������� ��������� ���������� � �������� � 1 V
					lcd_goto(2,0);
					lcd_prints("dBV: ");
					lcd_goto(2,5);
					dtostrf(fval,1,4,mval);
					// �������������� float � char
					lcd_prints(mval);														
					break;
				case 2: // ������ U � dBu
					lcd_goto(1,0);
					lcd_prints("Uin: ");
					lcd_goto(1,5);
					fval=(val/adc_counter)*0.0025;
					// Umax = [1023*2.56 V]/1024 = 2.5575 V
					// K = 2.5575 V/1023 = 0.0025
					dtostrf(fval,1,4,mval);
					// �������������� float � char
					// [float, ���. ������ ������ (� ��. "." � �����), ��. ����� ".", char]
					lcd_prints(mval);
					fval=20*log10(fval/0.775);
					// dBu: ���������� 20 ���������� ���������� ��������� ���������� � �������� � 0.775 V
					lcd_goto(2,0);
					lcd_prints("dBu: ");
					lcd_goto(2,5);
					dtostrf(fval,1,4,mval);
					// �������������� float � char
					lcd_prints(mval);															
					break;				
				default: break;
			}
			if((PIND&(1<<PD0))==0) // ���� �� PD0 ���������� 0 (������ ������), �� ������ state
			{
				_delay_ms(10); // ����������� ����������� ���������
				PORTD=PORTD|0x80; // OR = PB7 �������� ���. 1
				_delay_ms(250);
				PORTD=PORTD^0x80; // XOR = PB7 �������� ���. 0
				state++;				
					if(state>2)
					state=0;
			}
			adc_counter=0;
			val=0;			
		}
		_delay_ms(100);
		// �������� ����� ������� ���������� ��������
		*/
    }
	return 0;
}