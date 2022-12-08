/*
 * main.c
 *
 * Created: 11/30/2022 3:01:19 PM
 * Author : Brandon Egger
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHZ Clock
#endif

#define BAUD_RATE_230400_BPS 103 // BAUD = 9600 | (UBRRn = 16MHz / (16*9600-des. BAUD)) - 1

#include <avr/io.h>
#include <util/delay.h>

#include "lcd_display.h"
#include "fingerprint_reader.h"

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
	
	PORTB = PORTB & 0xF0;
}

/************************************************************************/
/* toggles whether lockbox is open                                      */
/************************************************************************/
uint8_t open_state = 0;
void setLockBoxDoor(uint8_t open) {
	if (open == 1) {
		open_state = 1;
		rotateStepper(0, 200);
	} else {
		open_state = 0;
		rotateStepper(1, 200);
	}
}

/************************************************************************/
/* toggles whether lockbox is open                                      */
/************************************************************************/
void toggleOpen() {
	if (open_state == 0) {
		setLockBoxDoor(1);
	} else {
		setLockBoxDoor(0);
	}
}

/************************************************************************/
/* Initializes proper pins for stepper controller                       */
/************************************************************************/
void initStepper() {
	DDRB = DDRB | 0x0f; // First four pins of PORTB set as outputs for stepper control
	PORTB = PORTB & 0xf0; // Set those pins to off
}

// Basic Application Flow Helpers

int main(void)
{
	// Runtime Values
	unsigned int ubrr = BAUD_RATE_230400_BPS;

	// Initializers
	UART_init(ubrr);
	initStepper();
	initLCD();
	startFPS();
	setFPSLED(0);
	
	deleteFingerPrint(0xFF);
    enrollFinger(0x01);
	
	uint8_t result;
	while (1) {
		if (open_state == 0) {
			displayLCDMessage("Tap finger", "to unlock");
		} else {
			displayLCDMessage("Tap finger", "to lock");
		}
		verifyFinger(0x01, &result);
		
		if (result >= 0x01) {
			displayLCDMessage("Success!", "");
			toggleOpen();
		} else {
			displayLCDMessage("ERROR: Finger", "not recognized");
			_delay_ms(5000);
		}
	}
	
}


