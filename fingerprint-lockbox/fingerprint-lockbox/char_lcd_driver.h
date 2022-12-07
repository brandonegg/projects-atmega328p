#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#ifndef F_CPU
#define F_CPU 16000000UL	// 16 MHz clock speed
#endif

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/cpufunc.h>

#define Clear_Display 0x01		// no args
#define Return_Home 0x02		// no args
#define Entry_Mode_Set 0x04		// inc/dec, disp_shift
#define Display_Control 0x08	// disp_ON, crsr_ON, blink_at_cursor
#define Cursor_Display_Shift 0x10	// disp/crsr, right/left
#define Function_Set 0x20		// 8/4_bit, 2/1_line, 10/8_height
#define Set_CG_Address 0x40		// CGRAM_address
#define Set_DD_Address 0x80		// DDRAM_address
#define Init_Function_8 0x3		// upper nibble of 8-bit setting
#define Init_Function_4 0x2		// upper nibble of 4-bit setting

#define E_HI { PORTC |= 1<<PORTC4; }
#define E_LO { PORTC &= ~(1<<4); }
#define RS_HI { PORTC |= 1<<PORTC5; }
#define RS_LO { PORTC &= ~(1<<5); }

#define DELAY_US(us) { __builtin_avr_delay_cycles(16*us); }
#define DELAY_MS(ms) { __builtin_avr_delay_cycles(16000*ms); }

// Higher-level functions:
void LCD_Setup(void);
void Message(unsigned char * str);
void Command(uint8_t cmd, uint8_t arg);
void Data(uint8_t data);

// Lower-level functions:
void Send_Byte(uint8_t output, uint8_t isLong);
void Send_Nibble(uint8_t output);
void Pulse_E(void);

#endif