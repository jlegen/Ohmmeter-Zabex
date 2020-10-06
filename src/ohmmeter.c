/*
 * Version 1.0 vom 27.04.2012 (Zabex)
 *
 * $Id: ohmmeter.c 426 2018-11-02 07:59:23Z axel $
 *
 *
 *                  ATMega48
 *                    --u--
 *    (Reset)  PC6 1 |     | 28 ADC5  U_PB3
 *      Hold   PD0 2 |     | 27 ADC4  U_PB2
 *        TX   PD1 3 |     | 26 ADC3  U_PB1
 * LCD_Enable  PD2 4 |     | 25 ADC2  U_PB6
 * LCD_RS      PD3 5 |     | 24 ADC1  U_PB0
 * LCD_Data4   PD4 6 |     | 23 ADC0  Ux Messspannung
 *             VCC 7 |     | 22 GND
 *             GND 8 |     | 21 AREF
 *      R220   PB6 9 |     | 20 AVCC
 *             PB7 10|     | 19 PB5   R2M0  (SCK)
 *  LCD_Data5  PD5 11|     | 18 PB4   R330K (MISO)
 *  LCD_Data6  PD6 12|     | 17 PB3   R47K  (MOSI)
 *  LCD_Data7  PD7 13|     | 16 PB2   R6K8
 *      R100   PB0 14|     | 15 PB1   R1K0
 *                    -----
 *
 * Spannungsversorgung: 5.0V
 * Interner RC Oszillator mit 1MHz  (Teiler/8 eingeschaltet)
 *
 */


#include <inttypes.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "display2x8.h"

#define POWER_ON   { PORTD |=  (1<<PD0); }
#define POWER_OFF  { PORTD &= ~(1<<PD0); }

//auto-off, multiples of ~262 ms
#define TIMEOUT 115


const uint8_t g_mux[7]= {
    0,
    0,
    5,
    4,
    3,
    2,
    1
};

const uint8_t g_mask[7]= {
    BIT5,
    BIT4,
    BIT3,
    BIT2,
    BIT1,
    BIT6,
    BIT0,
};

const float g_Rref[7]= {
    2002000.0,
    329600.0,
    46930.0,
    6790.0,
    1000.0,
    220.0,
    100.5
};

const uint16_t e24[24]= {
    10, 11, 12, 13, 15, 16,
    18, 20, 22, 24, 27, 30,
    33, 36, 39, 43, 47, 51,
    56, 62, 68, 75, 82, 91
};

const float Rmax= 99000000.0;

uint16_t g_inaccuracy;
volatile uint8_t g_powerdown;


uint16_t getAdc(void)
{
    //warte bis zu 100 Messungen
    //bis sich der Messwert stabilisiert
    uint8_t  cnt=  100;
    uint16_t last= 0xffff;

    while (cnt) {
        ADCSRA|= (1<<ADSC);
        while (ADCSRA & (1<<ADSC))
            ;
        if (ADC == last)
            break;
        last= ADC;
        --cnt;
    }

    //Summe von 32 Messungen
    uint16_t sum= 0;
    for (uint8_t n= 0; n<32; n++){
        ADCSRA|= (1<<ADSC);
        while (ADCSRA & (1<<ADSC))
            ;
        sum+= ADC;
    }
    return sum; //0..32736
}


float measureRx(uint8_t n)
{
    uint16_t U0= 32*1023;
    uint16_t Ux;

    if (n < 7) {
        PORTB= g_mask[n];
        DDRB=  g_mask[n];

        if (g_mux[n]>0) {
            ADMUX = (1<<REFS0) | g_mux[n];
            _delay_ms(1);
            U0 = getAdc();
        }

        ADMUX = (1<<REFS0);
        _delay_ms(1);
        Ux = getAdc();

        DDRB=  0;
        PORTB= 0;

        if (Ux >= 16*1023) {
            g_inaccuracy = Ux - (16*1023);
        } else {
            g_inaccuracy = (16*1023) - Ux;
        }

        if (U0 != Ux) {
            // Testwiderstand: Rx  Referenzwiderstand: Rref
            // Rx = Ux/Ix
            //    Strom durch Rx und Rref ist gleich:
            // Ix=Irref
            // Irref = Urref /Rref
            //    mit  Urref=U0-Ux  Spannung über Referenzwiderstand
            //    gibt das
            //Irref = (U0-Ux) / Rref
            //    damit ist der Testwiderstand:
            //Rx = Ux /((U0-Ux) / Rref)
            //    etwas umgeformt:
            //Rx = Ux * Rref / (U0 - Ux);
            float Rx= Ux;
            Rx *= g_Rref[n];
            Rx /= (U0 - Ux);
            return Rx;
        } else {
            return Rmax;
        }

    } else { //Spezialmessung mit 2 parallelen Referenzwiderständen
        PORTB= BIT0|BIT6; // 100+220 Ohm einschalten
        DDRB=  BIT0|BIT6;

        ADMUX = (1<<REFS0) | 1; //Uref= Vcc. Channel 1 (U über 125R)
        _delay_ms(1);
        uint16_t U1 = getAdc();

        ADMUX = (1<<REFS0) | 2; //Uref= Vcc. Channel 2 (U über 150R)
        _delay_ms(1);
        uint16_t U2 = getAdc();

        ADMUX = (1<<REFS0); //Uref= Vcc. Channel ADC0 (Ux)
        _delay_ms(1);  //Gib dem S+H Kondensator Zeit zum laden
        uint16_t Uxx = getAdc();

        DDRB=  0;
        PORTB= 0;

        if (Uxx >= (16*1023)) {
            g_inaccuracy = Uxx - (16*1023);
        } else {
            g_inaccuracy = (16*1023) - Uxx;
        }

        float R1= g_Rref[6]; //100R
        float R2= g_Rref[5]; //220R

        float Ixx = (U1-Uxx)/R1 + (U2-Uxx)/R2;  //Ixx = I1 + I2
        if (0!=Ixx) {
            float Rx = Uxx  / Ixx;
            return Rx;
        } else {
            return Rmax;
        }
    }
}


int32_t rasterung(float r)
{
    float diffmin= 10000000L;  //10M
    int32_t rr= 0;
    int32_t m= 1;

    for (uint8_t dekade=1; dekade<7; dekade++) {
        for (uint8_t i=0; i<24; i++) {
            int32_t val= e24[i] * m;
            float diff = val - r;
            if (diff < 0)
                diff = -diff;
            if (diff < diffmin) {
                diffmin= diff;
                rr= val;
            }
        }
        m*= 10;
    }
    return rr;
}


void displayResult(float y)
{
    uint8_t  c;
    uint16_t num;
    uint32_t frac;
    uint16_t l;
    uint32_t n;
    uint8_t leading0;
    uint32_t div;
    uint32_t r;
    uint8_t i;
    uint8_t z;
    uint16_t mult;
    uint8_t  dotpos;
    float yy;
    uint8_t cnt;

    if (y<10) {
        yy = y*10;
        mult=10;
    } else {
        yy=y;
        mult=1;
    }
    r= rasterung(yy);

    /*Typische Ausgaben:
     1R   3R3     9R1
     10R  47R    910R
     1K   1K2     9K1
     10K 220K    910K
     1M   4M7     9M1 */

    //Reduzieren auf 4 signifikante Stellen
    if (y < 10000000) {//10M
        g_powerdown= TIMEOUT;
        if (r >= 1000000) { //1M
            c='M';
            num =r/1000000;  //1..9M
            frac=r%1000000;
            l=frac/100000;   //eine Nachkommastelle
        } else if (r >= 1000) { //1K
            c='K';
            num=r/1000;
            frac=r%1000;
            l=frac/100;    //eine Nachkommastelle
        } else if (1==mult){
            c='R';
            num=r;
            l=0;
        } else {//Mult ==10
            c='R';
            num=r/10;
            l=r%10;
        }

        home1Lcd();
        lcdWrite(' ');
        //-- schreibe Kurzform, z.B.  4K7 -------------
        n=num;
        leading0 = 1;
        div=100;
        for (i=0; i<3; i++) {
            z=n/div;
            n%=div;
            div/=10;
            if ((z>0) || (0==leading0)) {
                lcdWrite9(z);
                leading0=0;
            } else {
                lcdWrite(' ');
            }
        }
        lcdWrite(c);
        if (l>0) {
            lcdWrite9(l);
        } else {
            lcdWrite(' ');
        }

        home2Lcd();


        //-- schreibe Messwert, z.B.  4697 -------------

        if (y<10) {
            n=y*100;
            dotpos=5;
        } else if (y<100){
            n=y*10;
            dotpos=6;
        } else {
            n=y;
            dotpos=7;
        }
        leading0 = 1;
        div=1000000;
        cnt=0;
        for (i=0; i<7; i++) {
            if (dotpos==i) lcdWrite(',');
            z=n/div;
            n%=div;
            div/=10;
            if ((z>0) || (0==leading0) || (i>=(dotpos-1))) {
                if (cnt++ < 4){ //Nur 4 signifikante Stellen
                    lcdWrite9(z);
                } else {
                    lcdWrite('0');
                }
                leading0=0;
            } else {
                if ((1==mult) || (i>1)) lcdWrite(' ');
            }
        }
    }
    else {
        if (TIMEOUT-4 == g_powerdown) lcdInit();
        clearLcd();
        sendStringToLCD("  ----");
    }
}


ISR(TIMER0_OVF_vect)
{
    if (g_powerdown) {
        g_powerdown--;
    } else {
        POWER_OFF;
    }
}


int main(void){

    DDRB= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6;
    DDRC= 0;
    DDRD= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;

    PORTD= 0;
    POWER_ON;
    g_powerdown= TIMEOUT;

    /* ADC at 62KHz, */
    ADCSRA= ((1<<ADEN)|(1<<ADPS2));  // 1MHz/16
    ADMUX=  (1<<REFS0)|14; //Uref= Vcc. Channel 14: interne 1.1V (Vbg)
    DIDR0=  BIT0|BIT1|BIT2|BIT3|BIT4|BIT5; //Digital inputs disabled

    TCCR0A = 0;
    TCCR0B = (1<<CS02)|(1<<CS00); //1MHz/1024=976Hz
    TIMSK0 = (1<<TOIE0); //Timer overflow every 262ms (3.81Hz)

    lcdInit();
    sei();  //Interrupts an

    //Messe Versorgungsspannung bis mind. 4.8V erreicht
    // Bei 4,8V=Uref liefert der ADC 1,1/4,8*1023*32=7502
    // Bei 4,9V=Uref liefert der ADC 1,1/4,9*1023*32=7348
    // Bei 5.0V=Uref liefert der ADC 1,1/5,0*1023*32=7201
    if (getAdc() > 7500){
        sendStringToLCD("Low Batt");
    }

    //Falls 4.8V nicht erreicht werden, schaltet der Timer ab
    while (getAdc() > 7500)
        ; //Do nothing  - wait for power

    lcdInit(); //Nochmal initialisieren, da jetzt erst die Versorgungsspannung optimal ist
    clearLcd();
    sendStringToLCD("Ohmmeter");
    home2Lcd();
    sendStringToLCD("/XL 2018");
    _delay_ms(1000);

    /*
     Algorithmus:
     Der Prüfling zieht den Meßspannungseingang auf GND.
     Nacheinander werden 8 verschiedene Pull-Up Widerstände eingeschaltet
     und die Spannung über dem Prüfling und über dem Referenzwiderstand 32
     mal gemessen. Das ist notwendig, da über dem  Innenwiderstand der
     Ports z.B. bei 330R Testwiderstand glatt 200mV abfallen.

     Derjenige Meßwert der am nächsten an 512*32 (512=Uref/2) ist, müßte der
     genaueste Messwert sein. Der wird dann angezeigt.

     Spannungsversorgung:
     Ein Taster schaltet die Akkuspannung auf ENABLE des MCP1640
     Der µC setzt direkt nach dem loslaufen PD2 auf H und hält so
     den MCP1640 eingeschaltet. Wenn g_powerdown im Timerinterrupt
     auf 0 runtergezählt wurde, schaltet sich der µC selber ab.

     Die Schaltung braucht im Betrieb (ohne Rx) ca. 4,2mA.
     Mit ausgeschaltetem Spannungsregler unter 1µA.

     */

    uint16_t inaccuracy_min;
    float bestRx;

    while (1) {
        inaccuracy_min= 16*1000;
        bestRx= Rmax;
        for (uint8_t i= 0; i<8; i++){
            float Rx= measureRx(i);
            if (g_inaccuracy <= inaccuracy_min){
                inaccuracy_min= g_inaccuracy;
                bestRx= Rx;
            }
        }
        displayResult(bestRx);
        _delay_ms(100);
    }
}

