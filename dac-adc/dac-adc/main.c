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

void UART_puthex8(uint8_t val)
{
	// extract upper and lower nibbles from input value
	uint8_t upperNibble = (val & 0xF0) >> 4;
	uint8_t lowerNibble = val & 0x0F;

	// convert nibble to its ASCII hex equivalent
	upperNibble += upperNibble > 9 ? 'A' - 10 : '0';
	lowerNibble += lowerNibble > 9 ? 'A' - 10 : '0';

	// print the characters
	UART_putc(upperNibble);
	UART_putc(lowerNibble);
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

// BEGIN ADC
void start_adc_meas() {
	ADCSRA |= (1<<ADEN)  | (1<<ADSC);      // Enable ADC and Start the conversion
	while( !(ADCSRA & (1<<ADIF)) );       // Wait for conversion to finish
	ADCSRA |= (1<<ADIF);   // Clear ADIF,ADIF is cleared by writing a 1 to ADSCRA
}
// END ADC

#define ADC_Channel_0   0b00000000 // ADC0 Channel
int main(void)
{
	uint16_t ADC_10bit_Result = 0;
	unsigned int ubrr = BAUD_RATE_230400_BPS;
	char data[] = "Hello from ATmega428p  ";
	
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);// Select ADC Prescalar to 128,
	// 11.0598MHz/128 = 85KHz
	ADMUX = ADMUX & 0xF0;           //Clear MUX0-MUX3,Important in Multichannel conv
	ADMUX|= ADC_Channel_0;         // Select the ADC channel to convert,Aref pin tied to 5V
	
	UART_init(ubrr);
	
	while(1)
	{
		char buf[8]; //arbitrary size of 1 register for now
		UART_getLine(buf, 1);
		if (buf[0] == 'G') 
		{
			start_adc_meas();
			
			ADC_10bit_Result   =  ADC;
			UART_puts(data);
			unsigned char lower = (ADC_10bit_Result & 0xFF);
			UART_puthex8(ADCL);
		}
	}
}

