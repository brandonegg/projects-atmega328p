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
#define DAC_WRITE_ADDRESS		0x00

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "i2cmaster.h"

// START OF UART
/*
   The following are utilities for handling RS232 serial communication. Many of these functions
   are modifications from the following guide:
   http://www.rjhcoding.com/avrc-uart.php
 */

/************************************************************************/
/* Initializes UART                                                     */
/* - uint16_t ubrr: BAUD conversion                                     */
/************************************************************************/
void UART_init(uint16_t ubrr)
{
	UBRR0L = (uint8_t)(ubrr & 0xFF); // set baudrate in UBRR
	UBRR0H = (uint8_t)(ubrr >> 8);

	UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // enable the transmitter and receiver
	UCSR0C = 0x06;
}

/************************************************************************/
/* Sends character over UART                                            */
/* - unsigned char ubrr: Char to send                                   */
/************************************************************************/
void UART_putc(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0))); // wait for transmit buffer to be empty
	UDR0 = data; // load data into transmit register
}

/************************************************************************/
/* Puts multiple char over UART                                         */
/* - char* s: Array of characters to send                               */
/************************************************************************/
void UART_puts(char* s)
{
	while(*s > 0) UART_putc(*s++); // transmit character until NULL is reached
}

/************************************************************************/
/* Sends 8 bit number over UART                                         */
/* - uint8_t val: value to send                                         */
/************************************************************************/
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

/************************************************************************/
/* Receives character sent over UART                                    */
/************************************************************************/
char UART_getc(void)
{
	while(!(UCSR0A & (1 << RXC0))); // wait for data
	return UDR0; // return data
}

/************************************************************************/
/* Receives line of characters (ended by carriage return)               */
/* - char* buf: buffer to store line                                    */
/* - uint8_t n: size of buffer                                          */
/************************************************************************/
void UART_getLine(char* buf, uint8_t n)
{
	uint8_t bufIdx = 0;
	char c;

	// while received character is not carriage return
	// and end of buffer has not been reached
	do
	{
		c = UART_getc(); // receive character
		buf[bufIdx++] = c; // store character in buffer
	}
	while((bufIdx < n) && (c != '\r'));

	buf[bufIdx] = 0; // ensure buffer is null terminated
}

// BEGIN ADC

/************************************************************************/
/* Intitializes ADC on atmega328p. Set Vref to Vcc                      */
/************************************************************************/
void init_adc() {
	ADMUX |= (1<<REFS0); // Select Vref=AVcc
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set pre-scalar to 128 and enable ADC
}

/************************************************************************/
/* reads ADC value. Triggers a single conversion                        */
/* - uint8_t ADCchannel: channel of ADC                                 */
/************************************************************************/
void read_adc(uint8_t ADCchannel)
{
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F); //select ADC channel with safety mask
	ADCSRA |= (1<<ADSC); //single conversion mode
	while( ADCSRA & (1<<ADSC) ); // wait until ADC conversion is complete
}

/************************************************************************/
/* Converts ADC value (10-bit) to float value (voltage)                 */
/* - uint16_t* adcMeas: ref to adc measurement (10-bit)                 */
/* - float* result: ref to where float value should be stored           */
/************************************************************************/
void adc_to_voltage(uint16_t* adcMeas, float* result) {
	*result = (*adcMeas/1023.0f)*5.0f; // 5v = 1023 - 0 (10bit ADC).
}

/************************************************************************/
/* Reads ADC value, converts to float, then outputs value over serial.  */
/************************************************************************/
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

/************************************************************************/
/* Calls output_adc_meas() amount # of times, with delay (seconds)      */
/* between each call.                                                   */
/* - int amount: # of adc samples outputted                             */
/* - int delay: delay between sample in seconds                         */
/************************************************************************/
void sample_multiple_adc_meas(int amount, int delay) {
	uint8_t counter = 0;
	char out[10];
	
	while (counter < amount) {
		sprintf(out, "t=%d s, ", counter * delay);
		UART_puts(out);
		output_adc_meas();
		
		for(uint8_t i = 0; i < delay; i++) {
			_delay_ms(1000);
		}
		counter++;
	}
}

// I2C
#define DAC_ADDRESS 0b01011000 // See MAX data sheet, AD1 and AD0 are low which correlate to bits 1,2 of the DAC_ADDRESS BYTE

/************************************************************************/
/* Sends I2C command to DAC that sets output channel and output voltage */
/* - int channel: channel #, either 0 or 1                              */
/* - float output: output voltage, (8-bit 255 max) - scales to 5v       */
/************************************************************************/
void change_dac_output(int channel, float output) {
	uint8_t outputConv = (output/5.0f)*255; // convert float to value between 0-255.

	i2c_start_wait(DAC_ADDRESS+I2C_WRITE);
	i2c_write(0b00000000 + channel); // No reset, normal op. state, address 0
	i2c_write(outputConv); // Output byte, supports any 8-bit val
	i2c_stop();
}

// UTILITY

/************************************************************************/
/* Converts a string to its 8bit int equivalent                         */
/* -const char *s: number string to convert                             */
/************************************************************************/
uint8_t atou8(const char *s)
{
	uint8_t v = 0;
	while (*s && *s != '\n') { v = (v << 1) + (v << 3) + (*(s++) - '0'); }
	return v;
}

/************************************************************************/
/* Reads in the input arguments and converts to their integer form based*/
/* on command structure command,arg1,arg2,argi                          */
/* -char* buffer: command string buffer                                 */
/* -int* args: arg array, should allocated size of #args expected       */
/* -int size: # of args to search for                                   */
/************************************************************************/
uint8_t read_int_args(char* buffer, int* args, int size) {
	char tempBuff[7];
	uint8_t strIndex = 0;
	uint8_t argIndex = 0;
	uint8_t tempIndex;
	uint8_t val;
	
	argIndex = 0;
	strIndex = 0;
	while (argIndex < size) {
		tempIndex = 0;
		while (strIndex < 7 && buffer[strIndex] != ',') {
			tempBuff[tempIndex++] = buffer[strIndex];
			strIndex++;
		}
		while (tempIndex < 7) {
			tempBuff[tempIndex++] = '\n';
		}
		
		val = atou8(tempBuff);
		args[argIndex++] = val;
		strIndex++; // skip the comma
	}
	
	return strIndex;
}

/************************************************************************/
/* Reads in the input arguments and converts to their float form based  */
/* on command structure command,arg1,arg2,argi                          */
/* -char* buffer: command string buffer                                 */
/* -float* args: arg array, should allocated size of #args expected     */
/* -int size: # of args to search for                                   */
/************************************************************************/
void read_float_args(char* buffer, float* args, int size) {
	char tempBuff[7];
	uint8_t strIndex = 0;
	uint8_t argIndex = 0;
	uint8_t tempIndex;
	float val;
	
	argIndex = 0;
	strIndex = 0;
	while (argIndex < size) {
		tempIndex = 0;
		while (strIndex < 7 && buffer[strIndex] != ',') {
			tempBuff[tempIndex++] = buffer[strIndex];
			strIndex++;
		}
		while (tempIndex < 7) {
			tempBuff[tempIndex++] = '\n';
		}
		
		val = atof(tempBuff);
		args[argIndex++] = val;
		strIndex++; // skips the next comma
	}
}


/************************************************************************/
/* Reads in serial command until carriage stop reached                  */
/* -char* buff: buffer to store command string                          */
/* -uint8_t size: # of args to search for                               */
/************************************************************************/
void read_command(char* buff, uint8_t size) {
	char c[1];
	uint8_t index = 0;
	c[0] = UART_getc();
	
	while (index+1 < size && c[0] != 10) { // 10 = end of line character for arduino interpreter
		buff[index++] = c[0];
		c[0] = UART_getc();
	}

	c[index] = 10;
}

// COMMANDS

/************************************************************************/
/* Loads input with read_command() and controls main command logic      */
/************************************************************************/
void handle_input() {
	char command[8] = "";
	read_command(command, 8);

	if (command[0] == 'G') {
		output_adc_meas();
	} else if (command[0] == 'M') {
		int argBuff[2];
		read_int_args(&command[2], argBuff, 2); // command[2] since we don't want to include the start command M,
		if (argBuff[0] < 2 || argBuff[0] > 20) {
			UART_puts("ERROR: Your sample argument must be 2<=n<=20\n");
			return;
		}
		if (argBuff[1] < 1 || argBuff[1] > 10) {
			UART_puts("ERROR: Your dt argument must be 1<=n<=10\n");
			return;
		}
		
		sample_multiple_adc_meas(argBuff[0], argBuff[1]);
	} else if (command[0] == 'S') {
		int channelArg[1];
		uint8_t floatStartPoint = read_int_args(&command[2], channelArg, 1)+2; // +2 to adjust for already added offset of buffer
		
		float voltArg[1]; 
		read_float_args(&command[floatStartPoint], voltArg, 1);

		if (channelArg[0] > 1 || channelArg[0] < 0) {
			UART_puts("Channel must be 0 or 1\n");
			return;
		}
		if (voltArg[0] > 5.00f) {
			UART_puts("Voltage can't exceed 5V\n");
			return;
		}
		
		change_dac_output(channelArg[0], voltArg[0]);
		char out[16];
		sprintf(out, "DAC Channel %d set to %.2f V\n", channelArg[0], voltArg[0]);
		UART_puts(out);
	} else {
		UART_puts("Command not found!\n");
	}
}

// Main routine
int main(void)
{
	// Memory Assignment
	unsigned int ubrr = BAUD_RATE_230400_BPS;
	
	// Initializers:
	init_adc();      // Enables and configures ADC
	UART_init(ubrr); // Enables and configures UART serial com
	i2c_init();

	change_dac_output(0, 5.0f);
	change_dac_output(1, 5.0f);
	UART_puts("DAC Initialized to 5.00V\n");
	
	while(1)
	{
		handle_input();
		_delay_ms(1);
	}
}

