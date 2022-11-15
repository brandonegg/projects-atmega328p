/*
 * dac-adc.c
 *
 * Created: 11/9/2022 5:26:54 AM
 * Author : brand
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHZ Clock
#endif

#define _OPEN_SYS_ITOA_EXT
#define BAUD_RATE_230400_BPS 103 // BAUD = 9600 | (UBRRn = 16MHz / (16*9600-des. BAUD)) - 1
#define INPUT_BUFFER_LENGTH  8     // Maximum input buffer read.
#define ADC_CHANNEL_0   0b00000000 // ADC0 Channel

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

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

// BEGIN I2C


// BEGIN ADC
void init_adc() {
	ADMUX |= (1<<REFS0); // Select Vref=AVcc
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaller to 128 and enable ADC
}

void read_adc(uint8_t ADCchannel)
{
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F); //select ADC channel with safety mask
	ADCSRA |= (1<<ADSC); //single conversion mode
	while( ADCSRA & (1<<ADSC) ); // wait until ADC conversion is complete
}

void adc_to_voltage(uint16_t* adcMeas, float* result) {
	*result = (*adcMeas/1023.0f)*5.0f; // 5v = 1023 - 0 (10bit ADC).
}

void output_adc_meas() {
	read_adc(ADC_CHANNEL_0);
	
	uint16_t ADC_10bit_Result = 0;
	uint8_t lowADC = ADCL;
	uint8_t highADC = ADCH;

	ADC_10bit_Result = (highADC << 8) + lowADC;
	float voltage = 0.0f;
	adc_to_voltage(&ADC_10bit_Result, &voltage);

	char out[4];
	sprintf(out, "v=%.2f", voltage);
	
	UART_puts(out);
	UART_puts(" V\n");
}

// UTILITY
void read_args(char* argBuff, int size) {
	char c[1];
	UART_getLine(c, 1);
	int buffIndex = 0;
	
	while (c[0] != '\n') {
		argBuff[buffIndex++] = c[0];
		UART_getLine(c, 1);
	}
}

// COMMANDS
void handle_input(char* buf, uint8_t n) {
	if (buf[0] == 'G') {
		output_adc_meas();
	} else if (buf[0] == 'M') {
		char argBuff[10] = "";
		read_args(argBuff, 2);
		UART_puts(argBuff);
	}
}

// Main routine
int main(void)
{
	// Memory Assignment
	unsigned int ubrr = BAUD_RATE_230400_BPS;
	char buf[INPUT_BUFFER_LENGTH];
	
	// Initializers:
	init_adc();      // Enables and configures ADC
	UART_init(ubrr); // Enables and configures UART serial com
	
	while(1)
	{
		UART_getLine(buf, 1);
		handle_input(buf, INPUT_BUFFER_LENGTH);
	}
}

