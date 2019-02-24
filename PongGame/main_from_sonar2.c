#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "uart.h"


#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

// Initialize UART functions
void USART_init(void);
void USART_send( unsigned char data);
void USART_putstring(char* StringPtr);

volatile char disc = 0; // variable to toggle between discrete and continuous mode (0=continuous, 1=discrete)
int disc_ticks[8] = {95, 100, 113, 126, 142, 150, 169, 189}; //{60, 63, 71, 80, 90, 95, 106, 119};

//variables used to tune the moving average to smooth out the continous signal
int msize = 5;
int movavg[5];
int sum = 0;
int k = 0;

volatile unsigned int lead = 1;
volatile unsigned int stop = 0; //makes sure the 10us pulse only occurs once
volatile unsigned int begin; //stores time for beginning of response pulse from sonar sensor
volatile unsigned int end; //stores time for end of response pulse from sonar sensor
volatile unsigned int prin = 0;
volatile unsigned int overflows = 0;
volatile unsigned int overflows0 = 0; //overflows of Timer0

float v; //voltage from the ADC
int mv; //millivolt value for the voltage from the ADC

//dac pin outputs
int b2;
int b3;
int b4;
int dac;

//used to detect the press of the disc/continous toggle button
int buttonold = 0;
int buttonnew = 0;

long pw_min = 4700; //min width of pulse
long pw_max = 48000; //max width of pulse
long pitch_tick_min = 95; //pw_min is mapped to this min number of pitch ticks to produce the corresponding
//highest frequency in the range
long pitch_tick_max = 189; //pw_max is mapped correspondingly to the min frequency

volatile unsigned int pti = 0;
volatile unsigned int pitch_ticks = 60;

//buffers used to print things to serial monitor
char buff[16];
char buff1[16];

/////////////////////////////////////////////////////////////////////////////
// Note: All USART code was from the Internet, we did not write it ourselves
void USART_init(void){
	
	/*Set baud rate */
	UBRR0H = (unsigned char)(BAUD_PRESCALLER>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALLER;
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_send( unsigned char data)
{
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	
}

void USART_putstring(char* StringPtr){
	
	while(*StringPtr != 0x00){
		USART_send(*StringPtr);
	StringPtr++;}
	
}
/////////////////////////////////////////////////////////////////////

//fires when the value in timer1 equals the set value in the OCR1A register\
//used to send a single 10us pulse to the sonar sensor
ISR (TIMER1_COMPA_vect) {
	if(stop) {
		TCCR1B |= (1<<ICES1); // enable input capture (rising edge)
		TIMSK1 |= (0x01); // enable overflow interrupt
		TCCR1A = 0; // disable output compare
		TIMSK1 = 0; // kill everything, especially OCIE1A (output compare interrupt enable)
		TIMSK1 |= (1<<ICIE1); // enable input capture
	}
	stop = 1; //makes sure it only fires once (next time through loop this interrupt will be disabled)
}

//used to count overflow of the timer1 to deduce the accurate width of the response pulse
ISR(TIMER1_OVF_vect) {
	overflows++;
}

//used to determine the beginning and end times for the sonar response pulse rising and falling edges
ISR (TIMER1_CAPT_vect) {
	if (lead) {
		begin = ICR1; //if capturing rising edge, get the value stored in ICR1
		} else {
		end = ICR1; //if falling edge is captured, store the value stored in ICR1
		prin = 1; //let the main function know that it can calcuate the width of the pulse
		TIMSK1 &= ~(1<<ICIE1); //disable input capture register
	}
	lead = 0; //set to falling edge
	TCCR1B &= ~(1<<ICES1); //toggle to detect a falling edge
}

//initialize the variables used to
void send_pulse() {
	lead = 1; //initially look for rising edge
	overflows = 0; //intialize the overflows to 0 to count how many occur during the response pulse
	stop = 0; //set stop back to 0 to make sure the 10us pulse is only sent once.
	OCR1A = TCNT1+160; //set the value in OCR1A to fire 160 ticks = 10us in the future
}

//initilaze timer1
void timer1_init() {
	DDRB |= (1<<PINB1) | (1<<PINB2); //set PINB1 to an output
	DDRB &= ~(1<<PINB0); //set PINB0 to an input
	TCCR1A = 0x40; //enable output compare
	TCCR1B = (1<<CS10); //enable the timer no prescale
	TIMSK1 = 0x02; // enable the output compare interrupts
	sei(); //enable global interrupts
}

//initialize timer0
void timer0_init() {
	TCCR0B = (1<<CS02) | (1 << CS00); //sets 1024 prescaler
	TIMSK0 = (1 << TOIE0); //enable timer0 overflow interrupt
}

//map function obtained from Arduino website to mapo one range to another
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//count the number of timer0 overflows to determine when the pwm output signal is to be toggled
ISR (TIMER0_OVF_vect) {
	overflows0++;
}

//initilize timer2 for outputting the pwm signal
void pwm_init() {
	DDRD |= (1<< PIND3); //set OC2B as output
	TCCR2A |= 0x20; //use a non-inverting signal
	TCCR2A |= 0x03;  // use fast pwm
	TCCR2B |= 0x08; // waveform generation
	TCCR2B |= (1 << CS22); //prescaler 64

	OCR2A = pitch_ticks; //set the period
	OCR2B = OCR2A/2; //set the duty cycle
}

//function to sum elements in a vector
int summer(int nums[]) {
	int s;
	for(int i = 0; i < msize; i++) {
		s += nums[i];
	}
	return s;
}

//read the voltage at a given input pin
float readADC(int pin) {
	int mask = 0xF0;
	ADMUX = (ADMUX & mask); //turning on ADC0
	ADMUX |=(1<< REFS0); //internal reference
	ADMUX |= pin; //telling it which pin to read from
	ADCSRA |= (1<< ADSC) | (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2); //start, enable, prescale, prescale
	while(ADCSRA & (1 << ADSC)); //wait: ADCSRA becomes done when it goes to 0 [takes a few clock cycles]
	int foo = ADCL; //graph LSB
	int foo1 = ADCH; //grab MSB
	uint16_t combined = (foo1 << 8 ) | (foo & 0xff); //combine the values in the two registers
	float v = 5.0*(float)combined/1024; //convert the value to a voltage
	return v;
}

int main(void) {
	
	USART_init();
	timer1_init(); // for timer 1
	timer0_init(); // for timer 0
	pwm_init(); // for timer 2
	OCR1A = TCNT1 + 16;
	send_pulse(); //call the function

	DDRB |= 0b00011100; //turn on PB2, PB3, PB4 for writing
	PORTB |= 0b00011100; //set them high initially
	DDRD &= ~(1<<PIND7); //PIND7 is read
	while(1) {
		if (overflows0 == 1) { //everytime timer0 overflows
			buttonnew = PIND & (1<<PIND7); //get the value from
			if (buttonnew) {//if button new is on and the old button state was off
				if (buttonold == 0) {
					if (disc == 1) {
						disc = 0;
					}
					else {
						disc = 1;
					}
					sprintf(buff,"disc: %d \n", disc);
					USART_putstring(buff);
				}
			}
			buttonold = buttonnew;
			
			v = readADC(0); //read pin A0 voltage
			mv = (int)(1000*v); //convert it to millivolts
			dac = map(mv, 100, 3300, 0, 6); //map the voltage to 0-->6 for toggling the output pins to the DAC

			b2 = (dac>>0) & 1; //get the 0th bit
			b3 = (dac>>1) & 1; //get the 1st bit
			b4 = (dac>>2) & 1; //get the 2nd bit

			//turn each of them on if they are 1, and off if they are 0
			if(b2) {
				PORTB |= (1<<PINB2);
			}
			else { PORTB &= ~(1<<PINB2); }
			if(b3) {
				PORTB |= (1<<PINB3);
			}
			else { PORTB &= ~(1<<PINB3); }
			if(b4) {
				PORTB |= (1<<PINB4);
			}
			else { PORTB &= ~(1<<PINB4); }

			//send a pulse again to get another distance from the sonar sensor
			timer1_init();
			send_pulse();

			overflows0 = 0; //reset the overflows to 0
		}

		//if a value for the return pulse width is ready to be processed
		if(prin) {

			//math to calculate the true length of the return pulse
			if (end < begin) {
				overflows--;
			}
			unsigned int num = end - begin;
			unsigned long pulse_width = ((long)overflows * 65536u + (long)num);

			//get rid of any values that are way too high or low
			if (pulse_width > pw_max) {
				pulse_width = pw_max;
			}
			if (pulse_width < pw_min) {
				pulse_width = pw_min;
			}

			//if in continous mode
			if (disc == 0) {
				pitch_ticks = map(pulse_width, pw_min, pw_max, pitch_tick_min, pitch_tick_max);

				//do a little moving average math to keep the signal from flucuating too much
				movavg[k] = pitch_ticks;
				k++;
				if (k == msize) {k = 0;}
				pitch_ticks = summer(movavg)/msize;
				} else { //if in discrete mode
				pti = map(pulse_width, pw_min, pw_max, 0, 7);
				pitch_ticks = disc_ticks[pti];
			}

			//update the fast pwm period and duty cycle corresponding to the value just calculated
			OCR2A = pitch_ticks;
			OCR2B = pitch_ticks/2;
			prin = 0; //set prin back to zero to wait for the next values to be processed
		}
	}
}