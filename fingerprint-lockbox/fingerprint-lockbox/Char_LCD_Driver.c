#include "char_lcd_driver.h"

// Global variables for this file:
uint8_t cgram_addr = 0;
uint8_t ddram_addr = 0;
uint8_t read_buffer = 0;
uint8_t stringBuffer[16] =
{
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0
};

void LCD_Setup(void)
{
	DDRC = 0xFF;
	RS_LO;
	
	// 8-bit mode commands
	DELAY_MS(100);
	Send_Nibble(Init_Function_8);
	DELAY_MS(5);
	Send_Nibble(Init_Function_8);
	DELAY_US(200);
	Send_Nibble(Init_Function_8);
	DELAY_US(200);
	Send_Nibble(Init_Function_4);
	DELAY_MS(5);
	
	// 4-bit mode commands
	Command(Function_Set, (1<<3));
	Command(Clear_Display, 0);
	Command(Display_Control, (1<<2));
	Command(Entry_Mode_Set, (1<<1));
}

void Message(unsigned char * str)
{
	uint8_t i = 0;		// str index
	while ((str[i] != 0) && i < 16)
	{
		Data(str[i]);
		i++;
	}
}

void Command(uint8_t cmd, uint8_t arg)
{
	uint8_t isLong = 0;
	if (cmd < 4)
	{ isLong++; }
	else if (cmd == 0x40)
	{ cgram_addr = arg; }
	else if (cmd == 0x80)
	{ ddram_addr = arg; }
	Send_Byte((cmd | arg), isLong);
}

void Data(uint8_t data)
{
	RS_HI;
	Send_Byte(data, 0);
	RS_LO;
}

//-----------------------------

void Send_Byte(uint8_t output, uint8_t isLong)
{
	Send_Nibble(output >> 4);
	Send_Nibble(output & 0xF);
	if (isLong)
	{ DELAY_MS(2); }
	else
	{ DELAY_US(45); }
}

void Send_Nibble(uint8_t output)
{
	PORTC = (PORTC && 0x30) | output;
	Pulse_E();
	DELAY_US(100);
}

void Pulse_E(void)
{
	E_HI;
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	E_LO;
}