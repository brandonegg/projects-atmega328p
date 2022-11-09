/*
 * dac-adc.c
 *
 * Created: 11/9/2022 5:26:54 AM
 * Author : brand
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHZ Clock
#endif

#define BAUD_RATE_230400_BPS  103 // BAUD = 9600 | (UBRRn = 16MHz / (16*9600-des. BAUD)) - 1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// START OF UART
/*
   The following are utilities for handling RS232 serial communication. Many of these functions
   are modifications from the following guide:
   http://www.rjhcoding.com/avrc-uart.php
   Also see:
   https://www.xanthium.in/how-to-avr-atmega328p-microcontroller-usart-uart-embedded-programming-avrgcc
 */
void UART_init(uint16_t ubrr)
{
	UBRR0L = (uint8_t)(ubrr & 0xFF); // set baudrate in UBRR
	UBRR0H = (uint8_t)(ubrr >> 8);

	UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // enable the transmitter and receiver
	UCSR0C = 0x06;
}

void UART_putc(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0))); // wait for transmit buffer to be empty
	UDR0 = data; // load data into transmit register
}

void UART_puts(char* s)
{
	while(*s > 0) UART_putc(*s++); // transmit character until NULL is reached
}

char UART_getc(void)
{
	while(!(UCSR0A & (1 << RXC0))); // wait for data
	return UDR0; // return data
}

void UART_getLine(char* buf, uint8_t n)
{
	uint8_t bufIdx = 0;
	char c;

	// while received character is not carriage return
	// and end of buffer has not been reached
	do
	{
		// receive character
		c = UART_getc();

		// store character in buffer
		buf[bufIdx++] = c;
	}
	while((bufIdx < n) && (c != '\r'));

	// ensure buffer is null terminated
	buf[bufIdx] = 0;
}
// END OF UART

// BEGIN I2C

// END OF I2C

int main(void)
{
	unsigned int ubrr = BAUD_RATE_230400_BPS;
	char data[] = "Hello from ATmega328p  ";
	
	UART_init(ubrr);
	
	while(1)
	{
		char buf[8]; //arbitrary size of 1 register for now
		UART_getLine(buf, 1);
		if (buf[0] == 'G') 
		{
			UART_puts(data);
		}
	}
}

