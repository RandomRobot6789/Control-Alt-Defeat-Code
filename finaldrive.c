/*
 * File:   driving2.c
 * Author: williammulberry
 *
 * Created on February 18, 2026, 10:03 AM
 */


#include "xc.h"
#pragma config FNOSC = LPFRC

enum drivemode {forward, left, right, uturn, stop};
int steps = 0;
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    
    steps = steps + 1;
    _OC1IF = 0;
    
}


int main(void) {
    _RCDIV = 0;
    OC1CON1 = 0; // sets all settings to 0 for 1 and 2 ALL for pin 14/RA6
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0b111; // system clock chosen
    OC1CON2bits.SYNCSEL = 0b11111; // output compare module
    OC1CON2bits.OCTRIG = 0; // he says just do this one
    
    OC2CON1 = 0; // same thing on pin 4/RB0
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111; //
    OC2CON2bits.SYNCSEL = 0b11111; //
    OC2CON2bits.OCTRIG = 0; //
    
    ANSA = 0;
    ANSB = 0;
    
    //Stepper 1/left pins
    _TRISA6 = 0; //pwm pin 14
    _LATA6 = 0;
    _TRISA0 = 0; //Disable pin left
    _LATA0 = 1;// start with it on
    _TRISA3 = 0;// direction pin
    _LATA3 = 0;
    
    //Stepper 2/right pins
    _TRISB0 = 0; // pwm pin 4
    _LATB0 = 0;
    _TRISB1 = 0;//disable pin
    _LATB1 = 1;
    _TRISB2 = 0;// direction pin
    _LATB2 = 0;
    
    //switch pin - pull down resistor
    _TRISB7 = 1;//input
            
    //set up pwm 
    OC1CON1bits.OCM = 0b110; //edge aligned pwm
    OC2CON1bits.OCM = 0b110;
    
    OC1RS = 1249; //frequency and duty cycle of each
    OC1R = 625;
    
    OC2RS = 1249;
    OC2R = 625;
    
    //interrupt setup
    _OC1IP = 4; // Select the interrupt priority
    _OC1IE = 1; // disable the OC1 interrupt for now
    _OC1IF = 0; // Clear the interrupt flag
    
    static enum drivemode mode = forward;

    static int i = 0;
    int leftval = 650;
    int forwardval = 1200;
    int uval = 550;
    
    while(1){
        switch(mode){
            case forward://enable steppers, same direction
                _LATA0 = 0;
                _LATB1 = 0;
                _LATA3 = 0;
                _LATB2 = 0;
                if(steps >= forwardval && i < 1){
                    mode = left;//need to turn it to left, reset the steps, set i to 1
                    steps = 0;
                    i++;
                }
                if(steps >= forwardval && i == 1){
                    mode = uturn;
                    steps = 0;
                    i++;               
                }
                if(steps >= forwardval && i > 1){
                    mode = stop;
                    steps = 0;
                 
                }
                if(_RB7 == 0){
                    mode = stop;
                    steps = 0;
                }
                break;
            case left://doing a one wheel pivot for this one, disable left stepper
                _LATA0 = 1;
                _LATB1 = 0;
                _LATA3 = 0;
                _LATB2 = 0;
                if(steps >= leftval){
                    mode = forward;
                    steps = 0;
                }
                if(_RB7 == 0){
                    mode = stop;
                    steps = 0;
                }
                break;
            case uturn://pivoting about the center, turn the right wheel reverse, enable both
                _LATA0 = 0;
                _LATB1 = 0;
                _LATA3 = 0;
                _LATB2 = 1;
                if(steps >= uval){
                    mode = forward;
                    steps = 0;
                }
                if(_RB7 == 0){
                    mode = stop;
                    steps = 0;
                }
                break;
            case stop:
                _LATA0 = 1;
                _LATB1 = 1;
                _LATA3 = 0;
                _LATB2 = 0;
                if(_RB7 == 1 && i == 0){
                    mode = forward;
                    steps = 0;
                    i = 0;
                }
    }

    }
    return 0;
}