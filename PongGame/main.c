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
//ACCELEROMETER VARIABLES
int accel_array[10];

//GAME VARIABLES
int max_score = 9;
long t = 0;
volatile unsigned int stop = 0; 
const char game = 1;

int touchX = 0;
int touchY = 0;
int touchIn = 0;

const int width = 2;
const int height = 10;
const int pitch_ticks = 1000;

const int top_of_arena = 1;
const int bottom_of_arena = 63;
const int left_side_of_arena = 0;
const int right_side_arena = 127;

char end_of_turn = 0;
char game_over = 0;

//int pastposx[1];
//int pastposy[1];
int ballvels[3] = {1, 2, 3};

//int lenpastpos = 1;

//GAME STRUCTS
struct ball_t{
	int x;
	int y;
	int r;
	int vx;
	int vy;
};

typedef struct ball_t ball;

struct player_t{
	int x;
	int y;
	int w;
	int h;
	int v;
	int score;
};
typedef struct player_t player;

// STRUCT INITIALIZATION
player player1 = { .x = 2, .y = 26, .w = 2, .h = 12, .v = 0, .score = 0};
player player2 = { .x = 124, .y = 26, .w = 2, .h = 12, .v = 0, .score = 0};
ball ball0 = {.x = 63, .y = 31, .r = 3, .vx = 1, .vy = 0};



//DRAW DEMO
int drawX = 0;
int drawY = 0;
int inxmin = 950;
int inxmax = 145;
int inymin = 150;
int inymax = 735;




//Read the analog voltage at a given pin
float readADC(int pin) {
	ADMUX &= ~((1<<MUX0) | (1<<MUX1)); //reset the pin selection pins
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

//check if the screen is being touched
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
	//draw arena
	drawrect(buff, 0,0,127, 63, 1);
	drawchar(buff, 54,7, 48 + player1.score);
	drawchar(buff, 67, 7, 48 + player2.score);
	
	//draw paddles
	fillrect(buff,player1.x, player1.y, player1.w, player1.h, 1);
	fillrect(buff,player2.x, player2.y, player2.w, player2.h, 1);
	
	//draw the ball (and past balls)
	//for (int i = 0; i < lenpastpos; i++) {
	drawball(buff, ball0.x, ball0.y, 1);
	//}

}

void serve() {
	//long r = rand() % 3;
	long r = 0;
	int sign = rand() % 2;
	if (sign==0) {
		sign=-1;
	}
	ball0.vx = sign*ballvels[r];
	//sprintf(strbuff, "range: %d \n", r);
	//USART_putstring(strbuff);
	//sprintf(strbuff, "veli: %ld \n", veli);
	//USART_putstring(strbuff);
	sprintf(strbuff, "vx: %d \n", ball0.vx);
	USART_putstring(strbuff);
}

void update_positions(long t) {
	ball0.x += ball0.vx;
	ball0.y += ball0.vy;
	//int i = t % lenpastpos;
	//pastposx[i] = ball0.x;
	//pastposy[i] = ball0.y;
	
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

ISR (TIMER1_COMPA_vect) {
	PORTB &= ~(1<<PINB3);
	//TCCR1A = 0; // disable output compare
	//TIMSK1 = 0; // kill everything, especially OCIE1A (output compare interrupt enable)
}
void timer1_init() {
	DDRB |= (1<<PINB3);  //set PINB1 to an output
	TCCR1A = 0x40; //enable output compare
	TCCR1B = (1<<WGM12) || (1<<CS10) || (1<<CS11); //enable the timer 64 prescale
	TIMSK1 = 0x02; // enable the output compare interrupts
	sei(); //enable global interrupts
}

void make_sound() {
	//stop = 0;
	PORTB|= (1<<PINB3); 
	TCNT1 = 0;
	OCR1A = TCNT1+50000; //set the value in OCR1A to fire 
	//TCCR1A |= 0x20; //use a non-inverting signal
	//TCCR1A |= 0x03;  // use fast pwm
	//TCCR1B |= 0x08; // waveform generation
	//TCCR1B |= (1 << CS22); //prescaler 64

	//OCR1A = pitch_ticks; //set the period
	//OCR1B = OCR1A/2; //set the duty cycle
}

int get_rand_velocity() {
	int random_value = rand() % 3;
	int sign = rand() % 2;
	if (sign==0) {
		sign=-1;
	}
	int velocity = sign*ballvels[random_value];
	return velocity;
}
void update_velocities() {
	//sprintf(strbuff, "VY: %d \n ", ball0.vy);
	//USART_putstring(strbuff);
	//sprintf(strbuff, "Y: %d \n", ball0.y);
	//USART_putstring(strbuff);
	if((ball0.x-ball0.r) -left_side_of_arena <= abs(ball0.vx)-1 ) {
		end_of_turn = 1;
		player2.score += 1;
		make_sound();
	} else if(right_side_arena-(ball0.x+ball0.r) <= abs(ball0.vx)-1 ) {
		end_of_turn = 1;
		player1.score += 1;
		make_sound();
	}

	if ((ball0.y-ball0.r) - top_of_arena <= abs(ball0.vy)-1) {
		ball0.vy = -ball0.vy;
		make_sound();
	} else if(bottom_of_arena  - (ball0.y+ball0.r) <= abs(ball0.vy)-1) {
		ball0.vy = -ball0.vy;
		//sprintf(strbuff, "FLAG \n");
		//USART_putstring(strbuff);
		make_sound();
	}

	if ((ball0.x-ball0.r) == (player1.x+1) && ball0.y < (player1.y+player1.h) && ball0.y > player1.y) {
		make_sound();
		int random_value = rand() % 3;
		ball0.vx = ballvels[random_value];
		ball0.vy = get_rand_velocity();
		//make_sound();
		} else if ((ball0.x+ball0.r) == (player2.x+1) && ball0.y < (player2.y+player2.h) && ball0.y > player2.y) {
		make_sound();
		int random_value = rand() % 3;
		ball0.vx = -1*ballvels[random_value];
		//make_sound();
		ball0.vy = get_rand_velocity();
	}
}

int get_average(int array[]) {
	int length = sizeof(array);
	int sum = 0;
	for(int i = 0; i<length; i++) {
		sum += array[i];
	}
	int average = sum/length;
	return average;
}

void get_accleromter_values() {
	int adc = readADC(4); //x value from accelerometer
	int n = 10;
	for(int i=0, j=1;i<n;i++,j++) {
		if(i==(n-1)) {
			accel_array[i]=adc;
			} else {
			accel_array[i]= accel_array[j];
		}
	}
	int saverage = get_average(accel_array);
	//determine the velocity (not sure how -x is distinguished form +x)
}
void twoplayers_move() {
	readTouch();
	drawX = map((long)touchX, (long)inxmin, (long)inxmax, 0, 127);
	drawY = map((long)touchY, (long)inymin, (long)inymax, 63, 0);
	//sprintf(strbuff, "x: %d \n", drawX);
	//USART_putstring(strbuff);
	//sprintf(strbuff, "y: %d \n", drawY);
	//USART_putstring(strbuff);
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
			sprintf(strbuff, "flag");
			USART_putstring(strbuff);
		}
	}
}

int main(void)
{
	USART_init();
	timer1_init();
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
	_delay_ms(1000); //10k
	clear_buffer(buff);
	write_buffer(buff);
	_delay_ms(1000);
	
	//char String[] = "Hello, world! \n";
	//sprintf(strbuff, String);
	//USART_putstring(strbuff);
	
	while (game) {
		
		//TODO:
		//Make choice menu
		//Player v. Player
		//Player v. Computer
		//Accelerometer v. Computer
		int choice = 0;
		char player_v_player[] = "PLAYER V. PLAYER";
		char player_v_computer[] = "PLAYER V. COMPUTER";
		char accel_v_computer[] = "ACCEL V. COMPUTER";
		drawstring(buff,25,2,player_v_player,sizeof(player_v_player));
		drawstring(buff,20,4,player_v_computer,sizeof(player_v_computer));
		drawstring(buff,22,6,accel_v_computer,sizeof(accel_v_computer));
		drawchar(buff,10,2*choice+2,62);
		write_buffer(buff);
		//wait for button presses
		char game_over = 0;
		int start_game = 0;
		DDRB &= ~(1<<PINB4);
		DDRB &= ~(1<<PINB5);
		while(start_game == 0) {
			_delay_ms(150);
			if(PINB & (1<<PINB5)) {
			} else {
			start_game = 1;
		}
			if(PINB & (1<<PINB4)) {
			} else {
				drawchar(buff,10,2*choice+2,32);
				choice++;
				if(choice == 3) {
					choice = 0;
				}
				drawchar(buff,10,2*choice+2,62);
			}
			write_buffer(buff);
		}
		while (game_over == 0) {
			ball0.x = 63;
			ball0.y = 31;
			ball0.vy = 0;
			serve();
			while (end_of_turn == 0) {
				touchIn = touch();
				if(touchIn > 0) {
					twoplayers_move();
				}
				else {
					player1.v = 0;
					player2.v = 0;
				}
				clear_buffer(buff);
				update_positions(t);
				update_velocities();
				drawStage();
				write_buffer(buff);
				_delay_ms(50); //determines frame rate of game
				t++;
			}
			_delay_ms(100);
			end_of_turn = 0;
			//TODO:
			//end of turn sequence
			//flash lights
			
			if (player1.score >= max_score || player2.score >= max_score) {
				game_over = 1;
				PORTB &= ~(1<<PINB2);
				PORTB |= (1<<PINB0);
				PORTD |= (1<<PIND7);
				_delay_ms(500);
				PORTB |= (1<<PINB2);
				PORTB &= ~(1<<PINB0);
				PORTD |= (1<<PIND7);
				_delay_ms(500);
				PORTB |= (1<<PINB2);
				PORTB |= (1<<PINB0);
				PORTD &= ~(1<<PIND7);
				_delay_ms(500);
				PORTB &= ~(1<<PINB2);
				PORTB &= ~(1<<PINB0);
				PORTD &= ~(1<<PIND7);
			}
		}
		clear_buffer(buff);
		write_buffer(buff);
		char String[] = "GAME OVER";
		drawstring(buff,40,3,String,sizeof(String));
		if (player1.score==max_score) {
			char winner[] = "PLAYER 1 WINS";
			drawstring(buff,30,4,winner,sizeof(winner));
		} else {
			char winner[] = "PLAYER 2 WINS";
			drawstring(buff,30,4,winner,sizeof(winner));
		}
		write_buffer(buff);
		_delay_ms(250);
		clear_buffer(buff);
		write_buffer(buff);
		_delay_ms(1000);
		player1.score = 0;
		player1.y = 26;
		player2.score = 0;
		player2.y = 26;
		//TODO:
		//end of game sequence
		//change color of LCDs
	}
}
