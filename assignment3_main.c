/*
 * File:   main.c
 * Author: UPDATE THIS WITH YOUR GROUP MEMBER NAMES OR POTENTIALLY LOSE POINTS
 * 
 * Created on: USE THE INFORMATION FROM THE HEADER MPLAB X IDE GENERATES FOR YOU
 */

// FBS
#pragma config BWRP = OFF               // Table Write Protect Boot (Boot segment may be written)
#pragma config BSS = OFF                // Boot segment Protect (No boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Code Flash Write Protection bit (General segment may be written)
#pragma config GCP = OFF                // General Segment Code Flash Code Protection bit (No protection)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Select (Fast RC oscillator (FRC))
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-Speed Start-up disabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = ON            // CLKO Enable Configuration bit (CLKO output disabled; pin functions as port I/O)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)

// FICD
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (PGC2/PGD2 are used for programming and debugging the device)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

// #pragma config statements should precede project file includes.

#include <xc.h>
#include <p24F16KA101.h>

#include "clkChange.h"
#include "uart.h"
#include "functions.h"

#define TESTSEND

#define BUTTON1 (PORTAbits.RA2 == 0)
#define BUTTON2 (PORTBbits.RB4 == 0)
#define BUTTON3 (PORTAbits.RA4 == 0)

enum states {
    FASTMODE = 0b10000,
    FASTMODE_PB1 = 0b10001,
    FASTMODE_PB2 = 0b10010,
    FASTMODE_PB3 = 0b10011,
    FASTMODE_COMB12 = 0b10100,
    FASTMODE_COMB13 = 0b10101,
    FASTMODE_COMB23 = 0b10111,
    PROGMODE = 0b100000,
    PROGMODE_PB1 = 0b100001,
    PROGMODE_PB2 = 0b100010,
    PROGMODE_PB3 = 0b100011,
};

enum states last_state = FASTMODE;
enum states current_state = FASTMODE;
uint8_t PB_last_state = 0b000;
uint8_t PB_current_state = 0b000;

uint16_t slow = 0;
uint8_t received; 

uint16_t blink_rate = 250;
uint16_t delay_count = 0;
uint8_t PB_event;
uint8_t stay_on = 0;
uint16_t var_x = 250;

int main(void) {
    
    AD1PCFG = 0xFFFF; /* keep this line as it sets I/O pins that can also be analog to be digital */
    newClk(500);
    
    CNinit();
    IOinit();
    
    InitUART2();
    TRISBbits.TRISB8 = 0;
    LATBbits.LATB8 = 0;
    
    T2CONbits.T32 = 0; // operate timer 2 as 16 bit timer
    //T2CONbits.TSIDL = 0; //operate in idle mode
    IEC0bits.T2IE = 1; //enable timer interrupt
    PR2 = 250; // count for 1 ms
    
    led_off();
    
    while(1) primary_loop();
    
    return 0;
}

// Timer 2 interrupt subroutine
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void){
    delay_count++;
    TMR2 = 0;
    
    IFS0bits.T2IF = 0; // clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){
    IFS0bits.T3IF = 0; // clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    IFS1bits.CNIF = 0; // clear interrupt flag
    
    PB_last_state = PB_current_state;
    PB_current_state = (BUTTON1) | (BUTTON2 << 1) | (BUTTON3 << 2);
    
    if (current_state & 0b10000) {
        if (PB_last_state == 0b001 && PB_current_state == 0b000) 
            current_state = (current_state == FASTMODE_PB1 || stay_on == 1) ? FASTMODE : FASTMODE_PB1;
        else if (PB_last_state == 0b010 && PB_current_state == 0b000) 
            current_state = (current_state == FASTMODE_PB2  || stay_on == 1) ? FASTMODE : FASTMODE_PB2;
        else if (PB_last_state == 0b100 && PB_current_state == 0b000) 
            current_state = (current_state == FASTMODE_PB3  || stay_on == 1) ? FASTMODE : FASTMODE_PB3;
        else if (PB_current_state == 0b011) current_state = FASTMODE_COMB12;
        else if (PB_current_state == 0b101) current_state = FASTMODE_COMB13;
        else if (PB_current_state == 0b110) current_state = FASTMODE_COMB23;
        else if (PB_current_state == 0b111) {
            current_state = PROGMODE;
        }
    } else if (current_state & 0b100000) {
        if (current_state == PROGMODE_PB3) {
            U2STAbits.UTXEN = 1;
            while(U2STAbits.UTXBF==1);
            U2TXREG='\b';
            while(U2STAbits.TRMT==0);
        }
        if (PB_last_state == 0b001 && PB_current_state == 0b000) 
            current_state = (current_state == PROGMODE_PB1) ? PROGMODE : PROGMODE_PB1;
        else if (PB_last_state == 0b010 && PB_current_state == 0b000) 
            current_state = (current_state == PROGMODE_PB2) ? PROGMODE : PROGMODE_PB2;
        else if (PB_last_state == 0b100 && PB_current_state == 0b000) 
            current_state = (current_state == PROGMODE_PB3) ? PROGMODE : PROGMODE_PB3;
        else if (PB_current_state == 0b111) {
            current_state = FASTMODE; 
        }
    }
    
    //stay_on = 0;
    blink_rate = 0;
    
    PB_event = 1;
}
