/*
 * fingerprint-lockbox.c
 *
 * Created: 11/30/2022 3:01:19 PM
 * Author : brand
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHZ Clock
#endif

#include <avr/io.h>
#include <util/delay.h>

/************************************************************************/
/* Simple function for returning high and low byte of 16-bit value      */
/************************************************************************/
uint8_t getLowByte(uint16_t val) {
	return (val & 0xff);
}
uint8_t getHighByte(uint16_t val) {
	return (val >> 8) & 0xff;
}

// START OF LCD
/* The LCD is used to display helpful output to users of the lockbox
 */
/************************************************************************/
/* Set mode to either command sending (0) or character mode (1)         */
/************************************************************************/
void setMode(uint8_t mode) {
	if (mode == 1) {
		PORTC |= (1 << PINC4);
	} else {
		PORTC &= ~(1 << PINC4);
	}
}

/************************************************************************/
/* Pulses E pin for communication with LCD                              */
/************************************************************************/
void pulseE() {
	PORTC |= (1 << PINC5);
	_delay_ms(0.0025); // 250 us, (tech spec is 230us)
	PORTC &= ~(1 << PINC5);
}

/************************************************************************/
/* Send LCD 8-bit command, requires command mode set to work            */
/************************************************************************/
void sendLCDCommand(uint8_t command) {
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getHighByte(command);
	pulseE();
	_delay_ms(5); // 100us delay
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getLowByte(command);
	pulseE();
	_delay_ms(5);
}

/************************************************************************/
/* Display a single character inline, requires character mode set       */
/************************************************************************/
void displayLetter(char letter) {
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getHighByte(letter);
	pulseE();
	_delay_ms(0.1); // 100us delay
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getLowByte(letter);
	pulseE();
	_delay_ms(0.1);
}

/************************************************************************/
/* Initializes LCD inputs and output pins                               */
/************************************************************************/
void initLCD() {
	/* Basic pin specification
	 * PC0-3 mapped to D4-7 input of LCD
	 * PC4 - RS of LCD
	 * PC5 - E of LCD
	 */
	DDRC = 0xFF; // Set all pins of PORTC as outputs
	PORTC = 0x00; // Set all outputs to 0
	
	_delay_ms(100);
	setMode(0);
	
	uint8_t lcdRoutine[6] = {0x33, 0x32, 0x28, 0x01, 0x0c, 0x06};
	for (int i = 0; i < 6; i++) {
		sendLCDCommand(lcdRoutine[i]);
	}
}

void displayLetters(char* str) {
	setMode(1);
	
	while (*str != '\n') {
		displayLetter(*str);
		str++;
	}
}

int main(void)
{
	//TESTING STATUS L LED - PROGRAM MODE LIT
	DDRB = (1 << 5);
	PORTB = (1 << 5);

	initLCD();
	char test[5] = {'t', 'e', 's', 't', '\n'};
	
	displayLetters(test);
}


