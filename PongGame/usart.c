/////////////////////////////////////////////////////////////////////////////
// Note: All USART code was from the Internet, we did not write it ourselves
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#include "uart.h"


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