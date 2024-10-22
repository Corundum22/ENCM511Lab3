/*
 * File:   functions.c
 * Author: jarred
 *
 * Created on 18. Oktober 2024, 17:02
 */


#include "functions.h"
#include "uart.h"

extern enum states {
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

extern enum states last_state;
extern enum states current_state;
extern uint8_t PB_last_state;
extern uint8_t PB_current_state;

extern uint16_t slow;
extern uint8_t received; 

extern uint16_t blink_rate;
extern uint16_t delay_count;
extern uint8_t PB_event;
extern uint8_t stay_on;
extern uint16_t var_x;
uint16_t used_var_x = 250;


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

void unbreakable_delay_ms(uint16_t ms_count) {
    TMR2 = 0;
    delay_count = 0;
    T2CONbits.TON = 1;
    
    while (delay_count < ms_count) Idle();
    
    T2CONbits.TON = 0;
    PB_event = 0;
    blink_rate = 0;
    stay_on = 0;
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

void primary_loop() {
    blink_rate = 0;
        
        unbreakable_delay_ms(130); // delay without breaking to let inputs stabilize
        
        switch (current_state) {
            case FASTMODE:
                blink_rate = 0;
                Disp2String("Fast Mode: Idle");
                stay_on = 0;
                break;
            case FASTMODE_PB1:
                blink_rate = 250;
                Disp2String("Fast Mode: PB1 was pressed");
                stay_on = 0;
                break;
            case FASTMODE_PB2:
                blink_rate = 500;
                Disp2String("Fast Mode: PB2 was pressed");
                stay_on = 0;
                break;
            case FASTMODE_PB3:
                blink_rate = 1000;
                Disp2String("Fast Mode: PB3 was pressed");
                stay_on = 0;
                break;
            case FASTMODE_COMB12:
                stay_on = 1;
                Disp2String("Fast Mode: PB1 and PB2 were pressed");
                break;
            case FASTMODE_COMB13:
                stay_on = 1;
                Disp2String("Fast Mode: PB1 and PB3 were pressed");
                break;
            case FASTMODE_COMB23:
                stay_on = 1;
                Disp2String("Fast Mode: PB2 and PB3 were pressed");
                break;
            case PROGMODE:
                blink_rate = 0;
                Disp2String("Prog Mode: IDLE");
                stay_on = 0;
                break;
            case PROGMODE_PB1:
                Disp2String("Prog Mode: PB1 was pressed\n");
                stay_on = 0;
                blink_rate = 3000;
                break;
            case PROGMODE_PB2: 
                blink_rate = used_var_x;
                Disp2String("Prog Mode: PB2 was pressed, Setting = X");
                stay_on = 0;
                break;
            case PROGMODE_PB3: 
                blink_rate = 125;
                Disp2String("Prog Mode: Blink setting = X:  ");
                while (current_state == PROGMODE_PB3) {
                    delay_ms(blink_rate);
                    led_toggle();
                }
                stay_on = 0;
                used_var_x = var_x;
                break;
            default: current_state = FASTMODE;
        }
        
        
        /*
         * enter this loop only if blink_rate == 0 due to the state machine
         */
        if (stay_on) {
            led_on();
            Idle();
            //Disp2String("Left stay_on; ");
        }
        else if (blink_rate == 0) {
            led_off();
            Idle();
            //Disp2String("Left blink_rate == 0; ");
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
}
