/*
 * File:   linefollow.c
 * Author: williammulberry
 *
 * Created on February 28, 2026, 4:13 PM
 */




#include "xc.h"
void config_pins_for_ad(void)
{
    // Put all relevant registers into a known state
    TRISA = 0;
    TRISB = 0;
    ANSA = 0;
    ANSB = 0;
    //REMOVE ALL PINS EXCEPT FOR THE ONES FOR THE QRD
    // Configure the tri state adapter for input for all the analog pins
    TRISAbits.TRISA0 = 1; // Configure pin 2 for input
    //TRISAbits.TRISA1 = 1; // Configure pin 3 for input
    //TRISBbits.TRISB0 = 1; // Configure pin 4 for input
    //TRISBbits.TRISB1 = 1; // Configure pin 5 for input    
    //TRISBbits.TRISB2 = 1; // Configure pin 6 for input
    TRISAbits.TRISA2 = 1; // Configure pin 7 for input LEFT SENSOR
    TRISAbits.TRISA3 = 1; // Configure pin 8 for input MIDDLE SENSOR
    TRISBbits.TRISB4 = 1; // Configure pin 9 for input RIGHT SENSOR
    //TRISBbits.TRISB12 = 1; // Configure pin 15 for input    
    //TRISBbits.TRISB13 = 1; // Configure pin 16 for input
    //TRISBbits.TRISB14 = 1; // Configure pin 17 for input
    //TRISBbits.TRISB15 = 1; // Configure pin 18 for input
    
    // Configure all the analog pins for analog (as opposed to digital)
    ANSAbits.ANSA0 = 1; // Configure pin 2 for analog
    //ANSAbits.ANSA1 = 1; // Configure pin 3 for analog
    //ANSBbits.ANSB0 = 1; // Configure pin 4 for analog
    //ANSBbits.ANSB1 = 1; // Configure pin 5 for analog    
    //ANSBbits.ANSB2 = 1; // Configure pin 6 for analog
    ANSAbits.ANSA2 = 1; // Configure pin 7 for analOG LEFT SENSOR
    ANSAbits.ANSA3 = 1; // Configure pin 8 for analog MIDDLE SENSOR
    ANSBbits.ANSB4 = 1; // Configure pin 9 for analog RIGHT SENSOR
    //ANSBbits.ANSB12 = 1; // Configure pin 15 for analog    
    //ANSBbits.ANSB13 = 1; // Configure pin 16 for analog
    //ANSBbits.ANSB14 = 1; // Configure pin 17 for analog
    //ANSBbits.ANSB15 = 1; // Configure pin 18 for analog
}

// This function configures the A/D to read from
// multiple channels in scan mode.
void config_ad(void)
{
    
    _ADON = 0;    // AD1CON1<15> -- Turn off A/D during config
    
    // AD1CON1 register
    _ADSIDL = 0;  // AD1CON1<13> -- A/D continues in idle mode
    _MODE12 = 1;  // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;    // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;    // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;    // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;   // AD1CON2<15:14> -- Use VDD as positive
                  // ref voltage
    _NVCFG = 0;   // AD1CON2<13> -- Use VSS as negative
                  // ref voltage
    _BUFREGEN = 1;// AD1CON2<11> -- Result appears in buffer
                  // location corresponding to channel
    _CSCNA = 1;   // AD1CON2<10> -- Scans inputs specified
                  // in AD1CSSx registers
    _SMPI = 3;	  // AD1CON2<6:2> -- Every 9th conversion sent (number of channels sampled -1)
                  // to buffer (if sampling 10 channels)
    _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only

    //_CN2PDE = 1; 
    
    // AD1CON3 register
    _ADRC = 0;    // AD1CON3<15> -- Use system clock
    _SAMC = 0b00001;    // AD1CON3<12:8> -- Auto sample every A/D
                  // period TAD
    _ADCS = 0b00000010;    // AD1CON3<7:0> -- TAD needs to be at least 750 ns. Thus, _ADCS = 0b00000010 will give us the fastest AD clock given a 4 MHz system clock.

    // AD1CSS registers
    AD1CSSL = 0b1110000000000001; // choose A2D channels you'd like to scan here.
    //this is doing pins AN 0,13,14,15 which is pins 7,8,9
    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}

#pragma config FNOSC = FRCDIV
double leftval = 0; //pin 13
double midval = 0; //read from pin 14
double rightval = 0; // pin 15
double goal = 2.25;
double driveval_mid = 0;
double driveval_I = 0;
double Ilimit = 70000;
double ki = 0;
double kp = 40000;
double maxval_OCR = 7999;//This is the fastest we want the steppers to move - 1000 steps/sec
double minval_OCR = 129999;
double standard_OCR = 89999;
int main(void) {
    config_pins_for_ad();
    config_ad();
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
    
    
    //Stepper 1/left pins
    _TRISA6 = 0; //pwm pin 14
    _LATA6 = 0;
    _TRISA3 = 0;// direction pin 5
    _LATA3 = 0;
    
    //Stepper 2/right pins
    _TRISB0 = 0; // pwm pin 4
    _LATB0 = 0;
    _TRISB2 = 0;// direction pin 6
    _LATB2 = 0;
    
    //switch pin 11- pull down resistor
    _TRISB7 = 1;//input
            
    //set up pwm 
    OC1CON1bits.OCM = 0b110; //edge aligned pwm
    OC2CON1bits.OCM = 0b110;
    
    OC1RS = 19999; //frequency and duty cycle of each
    OC1R = 10000;
    
    OC2RS = 19999;
    OC2R = 10000;
    
    T1CONbits.TON = 1; //Starts 16 bit timer
    T1CONbits.TCS = 0; // Chooses internal clock
    T1CONbits.TCKPS = 0b10; //prescaler option
    PR1 = 25000; //57422 change later
    TMR1 = 0; // sets the timer to zero
    
    while(1){
        
        //collect values for the pins
        if(TMR1 >= 5){
            midval = (double)ADC1BUF0/4095.0*3.3; // Pin 2
            driveval_mid = kp*(midval - goal);
            /*driveval_I += ki*(midval - goal)*.02;
            if(driveval_I >= Ilimit){
                driveval_I = Ilimit;
            }
            if(driveval_I <= Ilimit*-1){
                driveval_I = Ilimit*-1;
            }*/
            if(OC2RS - driveval_mid - driveval_I <= maxval_OCR){
                OC2RS = maxval_OCR;
            }
            else if(OC2RS - driveval_mid - driveval_I >= minval_OCR){
                OC2RS = minval_OCR;
            }
            else{
               OC2RS = standard_OCR - driveval_mid - driveval_I; //right wheel, decreasing the value speeds up the motor
            }
            OC2R = OC2RS/2;
        
            if(OC1RS + driveval_mid + driveval_I <= maxval_OCR){ //setting up min and max values, we will tune these
               OC1RS = maxval_OCR;
            }
            else if(OC1RS + driveval_mid + driveval_I >= minval_OCR){
                OC1RS = minval_OCR;
            }
            else{
                OC1RS = standard_OCR + driveval_mid + driveval_I; //left wheel, decreasing the value speeds up the motor
            }
            OC1R = OC1RS/2;
            }
        
        /*if(TMR1 >= 2500){
            if(midval > goal){
                OC1RS = minval_OCR;
                OC2RS = standard_OCR;
                OC1R = .5*OC1RS;
                OC2R = .5*OC2RS;
            }
            else{
                OC2RS = minval_OCR;
                OC1RS = standard_OCR;
                OC2R = .5*OC2RS;
                OC1R = .5*OC1RS;
            }
            TMR1 = 0;
        }*/
        
        

    }
    return 0;
    
}
