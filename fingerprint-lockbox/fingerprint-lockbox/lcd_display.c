/*
 * lcd_display.c
 *
 * Created: 12/07/2022 4:14:19 PM
 * Author : Brandon Egger
 */ 
#include "lcd_display.h"

uint8_t getLowNibble(uint8_t val) {
	return (val & 0xf);
}

uint8_t getHighNibble(uint8_t val) {
	return (val >> 4) & 0xf;
}

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
/* - uint8_t command: command to send                                   */
/************************************************************************/
void sendLCDCommand(uint8_t command) {
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getHighNibble(command);
	pulseE();
	_delay_ms(5); // 100us delay
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getLowNibble(command);
	pulseE();
	_delay_ms(5);
}

/************************************************************************/
/* Display a single character inline, requires character mode set       */
/* - char letter: letter to display at cursor                           */
/************************************************************************/
void displayLetter(char letter) {
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getHighNibble(letter);
	pulseE();
	_delay_ms(0.1); // 100us delay
	PORTC &= 0xF0; // Clear lower
	PORTC |= 0x0F & getLowNibble(letter);
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

/************************************************************************/
/* Clears the entire LCD display                                        */
/************************************************************************/
void clearDisplay() {
	// TODO
}

/************************************************************************/
/* Displays a string message to the LCD                                 */
/* - uint8_t line: 0 for line 1, 1 for line 2                           */
/* - char* str: Character string terminated with \0                     */
/************************************************************************/
void displayMessage(uint8_t line, char* str) {
	setMode(1);
	
	while (*str != '\0') {
		displayLetter(*str);
		str++;
	}
}