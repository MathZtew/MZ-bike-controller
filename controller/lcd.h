#include "Arduino.h"

#define BOARD_ID    0x65
#define ADDRESS     0x30
#define SET_BL		0x62
#define GET_BUT     0x05
#define LCD_CLR     0x60
#define LCD_WR      0x61

#define LCD_WRITE_WAIT  17

void lcdSetBlacklight(uint8_t value);
uint8_t readButtons();
void lcdClear();
void lcdGoToXY(uint8_t x, uint8_t y);
void lcdWriteChar(char character, uint8_t x, uint8_t y);
void lcdWrite(char* string);
void lcdWrite(int intVal);
void lcdWrite(float floatVal, uint8_t precision);
void lcdWrite(float floatVal, uint8_t precision, uint8_t size);