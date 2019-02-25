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
#include "lcd.h"

#define FREQ 16000000
#define BAUD 9600
#define HIGH 1
#define LOW 0
#define BUFFER 1024
#define BLACK 0x000001

char displayChar = 0;
char strbuff[20];

//GAME VARIABLES
long t = 0;

int touchX = 0;
int touchY = 0;
int touchIn = 0;

int p1score = 0;
int p2score = 0;

const int width = 2;
const int height = 10;

int pastposx[1];
int pastposy[1];

int lenpastpos = 1;


struct ball_t{
	int x;
	int y;
	int r;
	int vx;
	int vy;
};

int ballvels[3] = {1, 2, 3};


typedef struct ball_t ball;

struct player_t{
	int x;
	int y;
	int w;
	int h;
	int v;
};
typedef struct player_t player;

player player1 = { .x = 2, .y = 26, .w = 2, .h = 12, .v = 0};
player player2 = { .x = 124, .y = 26, .w = 2, .h = 12, .v = 0};
ball ball0 = {.x = 63, .y = 31, .r = 3, .vx = 1, .vy = 0};



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

void drawStage() {
	drawrect(buff, 0,0,127, 63, 1);
	drawchar(buff, 54,7, 48 + p1score);
	drawchar(buff, 67, 7, 48 + p2score);
	
	//draw paddles
	fillrect(buff,player1.x, player1.y, player1.w, player1.h, 1);
	fillrect(buff,player2.x, player2.y, player2.w, player2.h, 1);
	
	//draw the ball (and past balls)
	for (int i = 0; i < lenpastpos; i++) {
		drawball(buff, pastposx[i], pastposy[i], 1);
	}

}

void serve() {
	long r = rand()-1;
	int sign;
	sign = map(r, -1, RAND_MAX, 0,2);
	if (sign == 0) {
		sign = -1;
	}
	long veli;
	veli = map(r, -1, RAND_MAX, 0, 3);
	ball0.vx = sign*ballvels[veli];
	//sprintf(strbuff, "range: %d \n", r);
	//USART_putstring(strbuff);
	//sprintf(strbuff, "veli: %ld \n", veli);
	//USART_putstring(strbuff);
	sprintf(strbuff, "vx: %d \n", ball0.vx);
	USART_putstring(strbuff);
}

void newpos(long t) {
	ball0.x += ball0.vx;
	ball0.y += ball0.vy;
	int i = t % lenpastpos;
	pastposx[i] = ball0.x;
	pastposy[i] = ball0.y;
	
	player1.y += player1.v;
	player2.y += player2.v;
	//make sure it's not exiting the bounds
	if (player1.y < 1) {
		player1.y = 1;
	}
	else if (player1.y > 63 - player1.h) {
		player1.y = 63 - player1.h;
	}
	if (player2.y < 1) {
		player2.y = 1;
	}
	else if (player2.y > 63 - player2.h) {
		player2.y = 63 - player2.h;
	}
}

void twoplayers_move() {
	readTouch();
	drawX = map((long)touchX, (long)inxmin, (long)inxmax, 0, 127);
	drawY = map((long)touchY, (long)inymin, (long)inymax, 63, 0);
	if (drawX < 63) { //player 1
		if (drawY < 31) {
			//player1 moves up (decreases y)
			player1.v = -1;
		}
		else {
			player1.v = 1;
		}
	}
	else { //player 2
		if (drawY < 31) {
			//player1 moves up (decreases y)
			player2.v = -1;
		}
		else {
			player2.v = 1;
		}
	}
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
	sprintf(strbuff, String);
	USART_putstring(strbuff);
	
	serve();
	
	
	while (1)
	{
		
		touchIn = touch();		
		if(touchIn > 150) {
			twoplayers_move();
		}
		else {
			player1.v = 0;
			player2.v = 0;
		}
		clear_buffer(buff);
		newpos(t);
		drawStage();
		write_buffer(buff);
		_delay_ms(50);
		t++;

	}
}
