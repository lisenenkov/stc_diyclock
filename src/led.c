// LED functions for 4-digit seven segment led

#include "led.h"

const uint8_t ledtable[] = {
    // digit to led digit lookup table
    // dp,g,f,e,d,c,b,a 
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01100111  // 9
};

void filldisplay(uint8_t dbuf[4], uint8_t val, uint8_t pos) {
    // store display bytes, inverted    
    dbuf[pos] = ~(ledtable[val]);
    if (pos == 2) {
        // rotate third digit, by swapping bits fed with cba
        dbuf[pos] = dbuf[pos] & 0b11000000 | (dbuf[pos] & 0b00111000) >> 3 | (dbuf[pos] & 0b00000111) << 3;
    }
}