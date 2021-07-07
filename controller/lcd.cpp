/*
 * LCD16x2.cpp
 *
 * Modified from sourcecode by 
 * 2013 OLIMEX LTD <support@olimex.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include "lcd.h"
#include <Wire.h>

uint8_t X = 0;
uint8_t Y = 1;

uint8_t x_upd = 0;
uint8_t y_upd = 1;

char line1[17] = "ABCDEFGHIJKLMNOP";
char line2[17] = "XBCDEFGHIJKLMNOP";
uint16_t line1_new = 0xFFFF;
uint16_t line2_new = 0xFFFF;

uint32_t time_next_update = 0;

void lcdSetBlacklight(uint8_t value){
	Wire.beginTransmission(ADDRESS);
	Wire.write(SET_BL);
	Wire.write(value);
	Wire.endTransmission();
}

/**
 * Read the state of the 4 buttons.
 * @return      Bitmask with the 4 values: LSB - BUT1, MSB - BUT4
 */
uint8_t readButtons(){
    uint8_t buttons;

    Wire.beginTransmission(ADDRESS);
    Wire.write(GET_BUT);
    Wire.endTransmission();
    Wire.requestFrom((int)ADDRESS, 1);
    while(Wire.available() > 0)
        buttons = Wire.read();

    return buttons;
}

/**
 * Clear the LCD screen.
 */
void lcdClear(){
    Wire.beginTransmission(ADDRESS);
    Wire.write(LCD_CLR);
    Wire.endTransmission();
    delay(100);
}

/**
 * Position the cursor of the LCD to a given X and Y coordinates.
 * @param x     X coordinate
 * @param y     Y coordinate
 */
void lcdGoToXY(uint8_t x, uint8_t y){
    if(x > 16 || x < 1)
        return;
    else
        X = x - 1;

    if(y > 2 || y < 0)
        return;
    Y = y;
}

void lcdWriteChar(char character, uint8_t x, uint8_t y) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(LCD_WR);
    Wire.write(y);
    Wire.write(x);
    Wire.write(character);
    Wire.endTransmission();
}

/**
 * Write string to the LCD screen.
 * @param string        String to be written.
 */
void lcdWrite(char* string){
    //return;
    uint8_t len;
    uint8_t x, y;
    x = X;
    y = Y;

    len = strlen(string);
    for(int i = 0; i < len; i++){
        lcdWriteChar(string[i], x, y);
        delay(LCD_WRITE_WAIT);
        x++;
        if(x > 15){
            x = 0;
            y++;
            if(y > 2) {
                return;
            }
        }
    }
    X = x;
    Y = y;
}

void lcdWrite(int intVal){
    String Str = String (intVal);
    char charBuf[6];
    Str.toCharArray(charBuf, 6);
    lcdWrite(charBuf);
}

void lcdWrite(float floatVal, uint8_t precision){
    char charBuf[10];
    dtostrf(floatVal, 3, precision, charBuf);
    lcdWrite(charBuf);
}

void lcdWrite(float floatVal, uint8_t precision, uint8_t size){
    char charBuf[10];
    dtostrf(floatVal, 3, precision, charBuf);
    Serial.println(charBuf);
    bool next = false;
    for (int i = 0; i < size; i++) {
        if (!(charBuf[i] >= 0x2E && charBuf[i] <= 0x39) || next) {
            charBuf[i] = 0x20;
            next = true;
        }
    }
    charBuf[size] = 0;
    Serial.println(charBuf);
    lcdWrite(charBuf);
}