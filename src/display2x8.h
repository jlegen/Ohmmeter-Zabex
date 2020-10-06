#ifndef __display_h__
#define __display_h__

#include <inttypes.h>

#define BIT0 1
#define BIT1 2
#define BIT2 4
#define BIT3 8
#define BIT4 16
#define BIT5 32
#define BIT6 64
#define BIT7 128

void lcdInit(void);
void clearLcd(void);

void home1Lcd(void);
void home2Lcd(void);
void lcdWrite(uint8_t b);
void lcdWrite9(uint8_t n);

void sendStringToLCD(char *str);

#endif
