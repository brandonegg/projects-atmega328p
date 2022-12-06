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

/************************************************************************/
/* Simple function for returning high and low byte of 16-bit value      */
/************************************************************************/
uint8_t getLowByte(uint16_t val) {
	return (val & 0xff);
}
uint8_t getHighByte(uint16_t val) {
	return (val >> 8) & 0xff;
}

// START OF FPS
/*
   The finger print scanner used for this lab can be found here:
   https://learn.sparkfun.com/tutorials/fingerprint-scanner-gt-521fxx-hookup-guide?_ga=2.161334004.852986231.1669753998-1536484512.1668029572
 */
#define COMMAND_START_CODE_1 0x55
#define COMMAND_START_CODE_2 0xAA
#define COMMAND_DEVICE_ID_1 0x01
#define COMMAND_DEVICE_ID_2 0x00

/************************************************************************/
/* Sends command packet to FPS. Returns the 12 byte response (points to */
/* first byte)                                                          */
/* - uint16_t command: Command to send                                  */
/* - uint8_t* params: Parameters to send (must allocate 4 bytes)        */
/* - uint8_t* retBuff: Response buffer                                  */
/************************************************************************/
void sendFPSCommand(uint16_t command, uint8_t* params, uint8_t* retBuff) {
	// Compute checksum:
	uint16_t checkSumVal = COMMAND_START_CODE_1 + COMMAND_START_CODE_2 + COMMAND_DEVICE_ID_1 + COMMAND_DEVICE_ID_2 + command;
	checkSumVal += params[0] + params[1] + params[2] + params[3]; // Add the params

	uint8_t send[12] = {COMMAND_START_CODE_1, COMMAND_START_CODE_2, COMMAND_DEVICE_ID_1, COMMAND_DEVICE_ID_2, params[0], params[1], params[2], params[3], getLowByte(command), getHighByte(command), getLowByte(checkSumVal), getHighByte(checkSumVal)};
	UART_puts(send, 12);

	UART_getLine(retBuff, 12);
	/*
	 * this is an example of how you can check whether it actually received the command: 0x31 is not acknowledge (error), 0x30 is acknowledge
	if (buff[8] == 0x31) {
		PORTB = (1 << 5);
	}
	*/
}

/************************************************************************/
/* Shortcut for sending the FPS start command                           */
/************************************************************************/
void startFPS() {
	uint16_t command = 0x0001; // Start command 0x01
	uint8_t params[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
}

/************************************************************************/
/* Send start fingerprint enrollment command                            */
/* uint16_t id - ID the fingerprint will be stored under                */
/************************************************************************/
void startFingerEnroll(uint16_t id) {
	uint16_t command = 0x0022; // Enroll command 22
	uint8_t params[4] = {getLowByte(id), getHighByte(id), 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
	if (response[4] == 0x05 || response[8] == 0x31) { // Bad finger - 0x100C or 0x100D
		PORTB = (1 << 5); // there was error
	}
}

/************************************************************************/
/* Capture a finger print image. Must be done prior to enrollment step  */
/************************************************************************/
void captureFingerPrint() {
	uint16_t command = 0x60;
	uint8_t params[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
	if (response[4] == 0x31) {
		PORTB = (1 << 5); // there was error
	}
}

/************************************************************************/
/* delete a fingerprint. Use 0xFF to delete all fingerprints            */
/* uint16_t id: id to delete or 0xFF for all                            */
/************************************************************************/
void deleteFingerPrint(uint16_t id) {
	uint16_t command = 0x40;
	if (id == 0xff) {
		command += 1; // 0x41 to delete all
	}
	uint8_t params[4] = {getLowByte(id), getHighByte(id), 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
}

/************************************************************************/
/* Send start fingerprint enrollment command                            */
/* uint8_t step: enroll step (0-2)                                      */
/************************************************************************/
void handleEnrollStep(uint8_t step) {
	uint16_t command = 0x0023 + step; // Enroll 23 = Enroll 1, step is offset
	uint8_t params[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
	if (response[4] == 0x31) { // Bad finger - 0x100C or 0x100D
		PORTB = (1 << 5); // there was error
	}
}

/************************************************************************/
/* Shortcut for turning LED on/off                                      */
/************************************************************************/
void setLED(uint8_t on) {
	uint16_t command = 0x0012; // Start command 0x01
	uint8_t params[4] = {on, 0x00, 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
}

/************************************************************************/
/* Finger pressed                                                       */
/************************************************************************/
void waitForFingerPressed() {
	uint16_t command = 0x0026; // Start command 0x01
	uint8_t params[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
	while (response[4] != 0x00) { // response param 0 for finger pressed
		sendFPSCommand(command, params, response);
	}
}

/************************************************************************/
/* Finger released                                                      */
/************************************************************************/
void waitForFingerRelease() {
	uint16_t command = 0x0026; // Start command 0x01
	uint8_t params[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t response[12];
	
	sendFPSCommand(command, params, response);
	while (response[4] == 0x00) { // response param 0 for finger pressed
		sendFPSCommand(command, params, response);
	}
}

/************************************************************************/
/* Verify finger                                                        */
/* uint8_t step: finger id to verify                                    */
/* returns result                                                       */
/************************************************************************/
void verifyFinger(uint8_t id, uint8_t* output) {
	uint16_t command = 0x0050; // Start command 0x01
	uint8_t params[4] = {getLowByte(id), getHighByte(id), 0x00, 0x00};
	uint8_t response[12];
	
	setLED(1);
	waitForFingerPressed();
	captureFingerPrint();
	sendFPSCommand(command, params, response);
	waitForFingerRelease();
	setLED(0);
	
	if (response[4] == 0x00) {
		*output = 0x01;
	} else {
		*output = 0x00;
	}
}

/************************************************************************/
/* Handles the entire fingerprint enrollment flow                       */
/************************************************************************/
void enrollFinger(uint16_t id) {
	setLED(1);
	startFingerEnroll(id);
	
	for (int i = 0; i < 3; i++) {
		waitForFingerPressed();
		captureFingerPrint();
		handleEnrollStep(i);
		waitForFingerRelease();
	}
	
	setLED(0);
}

/* Stepper motor control:
 */
#define FIRST_STEP     0b0001
#define SECOND_STEP    0b0010
#define THIRD_STEP     0b0100
#define FOURTH_STEP    0b1000
#define STEP_DELAY     10 /* milliseconds between steps */

/************************************************************************/
/* Rotates the stepper amount in direction (0 for CW, 1 for CCW)        */
/************************************************************************/
void rotateStepper(uint8_t direction, uint16_t amount) {
	uint8_t pattern[4] = {FIRST_STEP, SECOND_STEP, THIRD_STEP, FOURTH_STEP};
	if (direction == 1) {
		pattern[0] = FOURTH_STEP;
		pattern[1] = THIRD_STEP;
		pattern[2] = SECOND_STEP;
		pattern[3] = FIRST_STEP;
	}

	for (uint16_t i = 0; i < amount; i++) {
		for (uint8_t j = 0; j < 4; j ++) {
			PORTB = pattern[j];
			_delay_ms(STEP_DELAY);
		}
	}
}

int main(void){
	DDRB = 0xff;    /* Enable output on all of the B pins */
	PORTB = 0x00;            /* Set them all to 0v */
	while(1){                     /* main loop here */
		rotateStepper(0, 200);
		rotateStepper(1, 200);
	}
}

/*
int main(void)
{
	//TESTING STATUS L LED
	DDRB = (1 << 5);
	PORTB = (0 << 5);
	
	unsigned int ubrr = BAUD_RATE_230400_BPS;
	UART_init(ubrr);

	startFPS();
	setLED(0);
	
	deleteFingerPrint(0xFF);
    enrollFinger(0x01);
	
	uint8_t result;
	while (1) {
		verifyFinger(0x01, &result);
		if (result >= 0x01) {
			PORTB = (1 << 5);
		} else {
			PORTB = (0 << 5);
		}
	}
	
}
*/

