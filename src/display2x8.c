// ----------------------------------------------------------------------------
//   LCD functions - 4 bit version (2 zeilig)
// ----------------------------------------------------------------------------

#include "display2x8.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define  LCD_ENABLE    { PORTD |=  BIT2; }
#define  LCD_DISABLE   { PORTD &= ~BIT2; }
#define  LCD_RS_ON     { PORTD |=  BIT3; }
#define  LCD_RS_OFF    { PORTD &= ~BIT3; }
#define  LCD_OUT(x)    { PORTD = (PORTD & 0x0F) | (x & 0xF0); }


void lcdWriteNibble(uint8_t b)
{
    asm volatile ("nop");
    LCD_ENABLE;
    LCD_OUT(b);
    _delay_us(4);
    LCD_DISABLE;
    _delay_us(2);
}


void lcdWrite(uint8_t b)
{
    lcdWriteNibble(b);
    lcdWriteNibble(b<<4);
    _delay_us(50);
}


void lcdWrite9(uint8_t n)
{
    lcdWrite(48+n);
}


static void cmd2Lcd(uint8_t cmd)
{
    LCD_RS_OFF;
    _delay_us(80);
    lcdWrite(cmd);
    LCD_RS_ON;
    _delay_us(2050);
}


void clearLcd(void)
{
    cmd2Lcd(0x01);
}


void home1Lcd(void)
{
    cmd2Lcd(2);
}


void home2Lcd(void)
{
    cmd2Lcd(192);
}


void lcdInit(void)
{
    LCD_RS_OFF;
    _delay_us(300);
    LCD_ENABLE;

    _delay_ms(20);

    lcdWriteNibble(0x30); // DB5=1 DB4=1
    _delay_ms(15);    // wait more than 4.1ms

    lcdWriteNibble(0x30); // DB5=1 DB4=1
    _delay_ms(15);    // wait more than 100us

    lcdWriteNibble(0x30); // DB5=1 DB4=1
    _delay_ms(15);    // wait more than 100us


    lcdWrite(0x28); //2 Lines, 4Bit Data,   0 0 1 DL  N F * *  DL:0=4Bit, 1=8Bit  n:0=1Zeile, 1:2Zeilen  F:0=5x7, 1=5x11 Pixel
    _delay_us(1000);    //just for paranoia
    lcdWrite(0x28); //2 Lines, 4Bit Data,
    _delay_us(1000);    //just for paranoia

    lcdWrite(0x08); // Display off
    _delay_us(1000);    //just for paranoia

    lcdWrite(0x0c); // Display on, Cursor + Blink off
    _delay_us(1000);    //just for paranoia

    lcdWrite(0x06); // Entry mode; no shift
    _delay_us(2000);

    LCD_RS_ON;
}


void sendStringToLCD(char *str)
{
    uint8_t i = 0;
    while (str[i] !=0)
    {
        lcdWrite(str[i]);
        i++;
        if (i > 7){
            break;
        }
    }
}

