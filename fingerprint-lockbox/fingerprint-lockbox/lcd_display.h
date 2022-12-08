/*
 * lcd_display.h
 *
 * Created: 12/07/2022 4:14:19 PM
 * Author : Brandon Egger
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHZ Clock
#endif

// Dependencies
#include <avr/io.h>
#include <util/delay.h>

// General utility
uint8_t getLowNibble(uint8_t);
uint8_t getHighNibble(uint8_t);

// Low level LCD controls
void setMode(uint8_t mode);
void pulseE();
void sendLCDCommand(uint8_t command);
void displayLetter(char letter);

// High level LCD controls
void initLCD();
void displayLCDMessage(char* line1, char* line2);
void clearLCD();