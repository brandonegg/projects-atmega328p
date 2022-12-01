/*
 * fingerprint-lockbox.c
 *
 * Created: 11/30/2022 3:01:19 PM
 * Author : brand
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHZ Clock
#endif

#define BAUD_RATE_230400_BPS 103 // BAUD = 9600 | (UBRRn = 16MHz / (16*9600-des. BAUD)) - 1

#include <avr/io.h>
#include <util/delay.h>

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
void UART_putc(uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0))); // wait for transmit buffer to be empty
	UDR0 = data; // load data into transmit register
}

/************************************************************************/
/* Puts multiple char over UART                                         */
/* - char* s: Array of characters to send                               */
/************************************************************************/
void UART_puts(uint8_t* s, int n)
{
	uint8_t i = 0;
	while(i < n) {
		UART_putc(*s++); // transmit character until NULL is reached
		i++;
	}
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
void UART_getLine(uint8_t* buf, uint8_t n)
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
	while(bufIdx < n);

	buf[bufIdx] = 0; // ensure buffer is null terminated
}

char getLowByte(char val) {
	return (val & 0xff);
}

char getHighByte(uint16_t val) {
	return (val >> 8) & 0xff;
}

// Finger print scanner
void startFPS() {
	uint8_t COMMAND_START_CODE_1 = 0x55;
	uint8_t COMMAND_START_CODE_2 = 0xAA;
	uint8_t COMMAND_DEVICE_ID_1 = 0x01;
	uint8_t COMMAND_DEVICE_ID_2 = 0x00;
	uint16_t command = 0x0001; // Start command 0x01
	uint8_t param[4] = {0x00, 0x00, 0x00, 0x00};
	
	uint16_t checkSumVal = COMMAND_START_CODE_1 + COMMAND_START_CODE_2 + COMMAND_DEVICE_ID_1 + COMMAND_DEVICE_ID_2 + command + param[0] + param[1] + param[2] + param[3];

	uint8_t send[12] = {COMMAND_START_CODE_1, COMMAND_START_CODE_2, COMMAND_DEVICE_ID_1, COMMAND_DEVICE_ID_2, param[0], param[1], param[2], param[3], getLowByte(command), getHighByte(command), getLowByte(checkSumVal), getHighByte(checkSumVal)};
	UART_puts(send, 12);
	
	uint8_t buff[12];
	UART_getLine(buff, 12);
	
	/*
	* this is an example of how you can check whether it actually received the command: 0x31 is not acknowledge (error), 0x30 is acknowledge
	if (buff[8] == 0x31) {
		PORTB = (1 << 5);
	}
	*/
}

void setLED(uint8_t on) {
	uint8_t COMMAND_START_CODE_1 = 0x55;
	uint8_t COMMAND_START_CODE_2 = 0xAA;
	uint8_t COMMAND_DEVICE_ID_1 = 0x01;
	uint8_t COMMAND_DEVICE_ID_2 = 0x00;
	uint16_t command = 0x0012; // Start command 0x01
	uint8_t param[4] = {on, 0x00, 0x00, 0x00};
	
	uint16_t checkSumVal = COMMAND_START_CODE_1 + COMMAND_START_CODE_2 + COMMAND_DEVICE_ID_1 + COMMAND_DEVICE_ID_2 + command + param[0] + param[1] + param[2] + param[3];

	uint8_t send[12] = {COMMAND_START_CODE_1, COMMAND_START_CODE_2, COMMAND_DEVICE_ID_1, COMMAND_DEVICE_ID_2, param[0], param[1], param[2], param[3], getLowByte(command), getHighByte(command), getLowByte(checkSumVal), getHighByte(checkSumVal)};
	UART_puts(send, 12);
	
	uint8_t buff[12];
	UART_getLine(buff, 12);
}

int main(void)
{
	//TESTING STATUS L LED
	//DDRB = (1 << 5);
	//PORTB = (0 << 5);
	
    /* Replace with your application code */
	unsigned int ubrr = BAUD_RATE_230400_BPS;
	UART_init(ubrr);

	startFPS();
	setLED(0);
	
    while (1) 
    {
		setLED(1);
		_delay_ms(1000);
		setLED(0);
		_delay_ms(1000);
    }
}

