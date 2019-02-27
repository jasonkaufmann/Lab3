#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "uart.h"
#include "usart.c"
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#include "lcd.c"

#define FREQ 16000000
#define BAUD 9600
#define HIGH 1
#define LOW 0
#define BUFFER 1024
#define BLACK 0x000001

char displayChar = 0;
char strbuff[20];

//GAME VARIABLES
int touchX = 0;
int touchY = 0;
int touchIn = 0;

//DRAW DEMO
int drawX = 0;
int drawY = 0;
int inxmin = 950;
int inxmax = 145;
int inymin = 150;
int inymax = 735;


float readADC(int pin) {
	ADMUX &= ~((1<<MUX0) | (1<<MUX1));
	ADMUX |= pin; //telling it which pin to read from, and turning on ADC
	ADMUX |= (1<< REFS0); //internal reference
	ADCSRA |= (1<< ADSC) | (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2); //start, enable, prescale, prescale
	while(ADCSRA & (1 << ADSC)); //wait: ADCSRA becomes done when it goes to 0 [takes a few clock cycles]
	return ADC;
}

//map function obtained from Arduino website to map one range to another
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int touch() {
	PORTC = 0;
	DDRC = 0;
	DDRC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3));
	//X- A0 -> HI-Z, Y+ A1 -> HI-Z (already from read mode)
	//X+ A2 -> ground
	//DDRC |= (1 << PINC2); 
	//PORTC &= ~(1 << PINC2);
	
	//check if Y- is high, A3
	int r = readADC(3);
	
	return r;
}


void readTouch() {
	//disable input capture
	//PORTC = 0;
	//DDRC = 0;
	
	//X- A0, X+ A2, Y- A3, Y+ A1
	//1. To read X coordinate first, we will put X- and X+ in digital mode and set X- high and X+ low.
	DDRC |= (1 << PINC0) | (1 << PINC2);
	PORTC |= (1 << PINC0);
	PORTC &= ~(1 << PINC2);
	//2. Then we set Y- , Y+ to ADC input mode and read the Y- ADC value as X coordinate on touchscreen
	DDRC &= ~((1 << PINC3) | (1 << PINC1));
	touchX = readADC(3);
	//3. Then we switch to read Y coordinate, for which we put Y- and Y+ to digital mode  and set Y+ to low and Y- to high.
	DDRC |= (1 << PINC3) | (1 << PINC1);
	PORTC |= (1 << PINC3);
	PORTC &= ~(1 << PINC1);
	//4. Set X- and X+ to ADC input, read X- ADC as y coord
	DDRC &= ~((1 << PINC0) | (1 << PINC2));
	touchY = readADC(0);
	//turn off ports
	//PORTC = 0;
	
}

int main(void)
{
	USART_init();
	//setting up the gpio for backlight
	DDRD |= 0x80;
	PORTD &= ~0x80;
	PORTD |= 0x00;
	
	DDRB |= 0x05;
	PORTB &= ~0x05;
	PORTB |= 0x00;
	
	//lcd initialisation
	lcd_init();
	lcd_command(CMD_DISPLAY_ON);
	lcd_set_brightness(0x18);
	write_buffer(buff);
	_delay_ms(100); //10k
	clear_buffer(buff);
	write_buffer(buff);
	_delay_ms(1000);
	
	char String[] = "Hello, world! \n";
	int four = 4;
	sprintf(strbuff, String);
	USART_putstring(strbuff);
	
	
			//drawrect(buff, 10, 5, 20, 50, 1);
			//fillcircle(buff, 70, 40, 15, 1);
			//drawstring(buff, 45, 0,"JASON AND B <3",14);
			//drawline_horz(buff, 50, 20, 90, 1);
			//write_buffer(buff);
	while (1)
	{
		//drawchar(buff,64,0,displayChar);
		//setpixel(buff, 100, 61, 1);
		//int size = sizeof(String);
		//fillcircle(buff, 64,32,20,1);
		//drawline(buff,1,0,50,5,1);
		//write_buffer(buff);
		//_delay_ms(1000); //5k
		//clearpixel(buff, 64, 32);
		touchIn = 951; //touch();
		
		if(touchIn > 950) {
			//sprintf(strbuff, "touch: %d ", touchIn);
			//USART_putstring(strbuff);
			readTouch();
			//sprintf(strbuff, "x: %d ", touchX);
			//USART_putstring(strbuff);
			//sprintf(strbuff, "   y: %d \n", touchY);
			//USART_putstring(strbuff);
			
			drawX = map((long)touchX, (long)inxmin, (long)inxmax, 0, 127);
			drawY = map((long)touchY, (long)inymin, (long)inymax, 63, 0);
			setpixel(buff, drawX, drawY, 1);
			write_buffer(buff);
		}

		

		_delay_ms(50);
		//displayChar++;

	}
}
