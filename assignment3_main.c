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
    FASTMODE_COMB123 = 0b11000,
    PROGMODE = 0b00000,
    PROGMODE_PB1 = 0b00001,
    PROGMODE_PB2 = 0b00010,
    PROGMODE_PB3 = 0b00011,
    PROGMODE_COMB123 = 0b00100,
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
uint8_t stay_on;
uint16_t var_x = 250;

void CNinit() {
    PB_event = 0;
    
    IPC4bits.CNIP = 6;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;
}

void IOinit() {
    TRISBbits.TRISB8 = 0;
    
    TRISAbits.TRISA4 = 1;
    CNPU1bits.CN0PUE = 1;
    CNEN1bits.CN0IE = 1;
    
    TRISBbits.TRISB4 = 1;
    CNPU1bits.CN1PUE = 1;
    CNEN1bits.CN1IE = 1;
    
    TRISAbits.TRISA2 = 1;
    CNPU2bits.CN30PUE = 1;
    CNEN2bits.CN30IE = 1;
}

void delay_ms(uint16_t ms_count) {
    TMR2 = 0;
    delay_count = 0;
    T2CONbits.TON = 1;
    
    while (delay_count < ms_count) {
        if (PB_event) {
            PB_event = 0;
            break;
        }
        Idle();
    }
    
    T2CONbits.TON = 0;
}

void led_on() {
    LATBbits.LATB8 = 1;
}

void led_off() {
    LATBbits.LATB8 = 0;
}

void led_toggle() {
    LATBbits.LATB8 ^= 1;
}

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
    
    while(1) {
        
    
        stay_on = 0;
        blink_rate = 0;
        switch (current_state) {
            case FASTMODE: blink_rate = 0; Disp2String("FASTMODE "); break;
            case FASTMODE_PB1: blink_rate = 250; Disp2String("FASTMODE_PB1 "); break;
            case FASTMODE_PB2: blink_rate = 500; Disp2String("FASTMODE_PB2 "); break;
            case FASTMODE_PB3: blink_rate = 1000; Disp2String("FASTMODE_PB3 "); break;
            case FASTMODE_COMB12: stay_on = 1; break;
            case FASTMODE_COMB13: stay_on = 1; break;
            case FASTMODE_COMB23: stay_on = 1; break;
            case FASTMODE_COMB123: current_state = PROGMODE_COMB123; break;
            case PROGMODE: blink_rate = 0; break;
            case PROGMODE_PB1: blink_rate = 3000; break;
            case PROGMODE_PB2: blink_rate = var_x; break;
            case PROGMODE_PB3: blink_rate = 125; break;
            case PROGMODE_COMB123: current_state = FASTMODE_COMB123; break;
            default: current_state = FASTMODE;
        }
        
        /*
         * enter this loop only if blink_rate == 0 due to the state machine
         */
        if (blink_rate == 0) {
            //Disp2String("Entered blink_rate == 0; ");
            led_off();
            Idle();
            //Disp2String("Left blink_rate == 0; ");
        }
        if (stay_on) {
            //Disp2String("Entered stay_on; ");
            led_on();
            Idle();
            //Disp2String("Left stay_on; ");
        }
        
        /*
         * enter this loop only if blink_rate != 0 due to the state machine
         * 
         * break out of this loop if the state changes
         */
        while (blink_rate != 0) {
            delay_ms(blink_rate);
            led_toggle();
        }
        
/*#ifdef TESTSEND
        XmitUART2('3',5);
        XmitUART2('\n',5);
        XmitUART2('\b',5);
        
        for (slow = 0; slow < 1000; slow++) {
            
        }
#endif*/
        
/*#ifndef TESTSEND
        Disp2String("Enter a character (hit enter to receive): ");
        received = RecvUartChar();
        Disp2String("\r\n you entered ");
        XmitUART2(received,1);
        XmitUART2('\r',1);
        XmitUART2('\n',1);
        
        
#endif*/
        
    }
    
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
        if (PB_last_state == 0b001 && PB_current_state == 0b000) current_state = (current_state == FASTMODE_PB1) ? FASTMODE : FASTMODE_PB1;
        else if (PB_last_state == 0b010 && PB_current_state == 0b000) current_state = (current_state == FASTMODE_PB2) ? FASTMODE : FASTMODE_PB2;
        else if (PB_last_state == 0b100 && PB_current_state == 0b000) current_state = (current_state == FASTMODE_PB3) ? FASTMODE : FASTMODE_PB3;
    }
    
    blink_rate = 0;
    
    PB_event = 1;
}