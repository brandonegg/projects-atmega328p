/*
 * fingerprint_reader.h
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

// FPS Command Codes
#define COMMAND_START_CODE_1 0x55
#define COMMAND_START_CODE_2 0xAA
#define COMMAND_DEVICE_ID_1 0x01
#define COMMAND_DEVICE_ID_2 0x00

// UART Related (may be necessary to move to its own file if more serial devices are needed
void UART_init(uint16_t ubrr);
void UART_putc(uint8_t data);
void UART_puts(uint8_t* s, int n);
void UART_puthex8(uint8_t val);
char UART_getc(void);
void UART_getLine(uint8_t* buf, uint8_t n);

// General utility
uint8_t getLowByte(uint16_t val);
uint8_t getHighByte(uint16_t val);

// Finger print reader (low-level)
void sendFPSCommand(uint16_t command, uint8_t* params, uint8_t* retBuff);
void handleEnrollStep(uint8_t step);
void setLED(uint8_t on);
void waitForFingerPressed();
void waitForFingerRelease();

// Finger print reader (high-level)
void startFingerEnroll(uint16_t id);
void startFPS();
void captureFingerPrint();
void deleteFingerPrint(uint16_t id);
void verifyFinger(uint8_t id, uint8_t* output);
void enrollFinger(uint16_t id);