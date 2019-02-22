#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include "lcd.h"

#define FREQ 16000000
#define BAUD 9600
#define HIGH 1
#define LOW 0
#define BUFFER 1024
#define BLACK 0x000001

char displayChar = 0;


int main(void)
{
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
	_delay_ms(3000); //10k
	clear_buffer(buff);
	write_buffer(buff);
	_delay_ms(3000);
	
	//char String[] = "Hello, world!";
	while (1)
	{
		//drawchar(buff,64,0,displayChar);
		//setpixel(buff, 100, 61, 1);
		//int size = sizeof(String);
		fillrect(buff, 1,10,50,50,1);
		//write_buffer(buff);
		//_delay_ms(1000); //5k
		//clearpixel(buff, 64, 32);
		write_buffer(buff);
		_delay_ms(10000);
		//displayChar++;
	}
}

