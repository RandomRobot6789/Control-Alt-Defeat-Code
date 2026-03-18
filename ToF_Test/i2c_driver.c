/*
 * File:   i2c_driver.c
 * Author: Andrew_Jenkins
 *
 * Created on February 3, 2026, 9:56 PM
 */


#include "xc.h"
#include "i2c_driver.h"
#include "settings.h"

#define I2C_TIMEOUT_CLOCK_CYCLE_NUM 0xFFFF //my math says ~830 for a 6-byte multiRead with a bit of margin

//SCL and SDA are pins 12 and 13, respectively
//I2C address is 0x52 by default for the VL53L0X
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    //pretty sure this means that the bus is hung. Gotta unstick it.
    IFS0bits.T1IF = 0b0;
    I2C1CONbits.I2CEN = 0b0;
    TRISBbits.TRISB8 = 0b0;
    for (int i=0; i<8; i++) {
        LATBbits.LATB8 = 0b0;
        Nop();
        Nop();
        LATBbits.LATB8 = 0b1;
        Nop();
        Nop();
    }
    TRISBbits.TRISB8 = 0b1;
    I2C1CONbits.I2CEN = 0b1;
}

//code to replace TwoWire and Wire.h:
void init_i2c() {
    I2C1CONbits.I2CEN = 0b1; //enable I2C
    //set i2c clock
    I2C1BRG = 0x0009; //generate clock at 385 kHz; (400kHz nominal); assumes 4MHz main clock
    IPC0bits.T1IP = 6; //interrupt priority
    T1CON = 0x8000; //no prescaling
    PR1 = I2C_TIMEOUT_CLOCK_CYCLE_NUM;
}

void start(uint8_t addr) { //this is called at the start of write transactions, requestfrom is for read transactions
    TMR1 = 0b0000;
    //IEC0bits.T1IE = 0b1; //start timeout timer
#ifdef OSC_I2C_START
    OSC_ON;
#endif
    I2C1CONbits.SEN = 0b1; //bus start
    while(I2C1CONbits.SEN) {}
    write((addr << 1) | 0x00); //write address
#ifdef OSC_I2C_START
    OSC_OFF;
#endif
}
void write(uint8_t byte_to_write) {
    I2C1TRN = byte_to_write;
    while(I2C1STATbits.TRSTAT) {
//        if(I2C1STATbits.BCL) {
//            I2C1CONbits.I2CEN = 0b0;
//            LATBbits.LATB8 = 0;
//            LATBbits.LATB8 = 1;
//            I2C1CONbits.I2CEN = 0b1;
//        }
    } //do I need to check for ACK separately? I can't tell
}
void stop() {
    I2C1CONbits.PEN = 0b1; //bus stop
    while(I2C1CONbits.PEN) {}
    IEC0bits.T1IE = 0b0; //transaction completed successfully, bus isn't hung
}


void requestFrom(uint8_t addr, uint8_t num_bytes) {
    TMR1 = 0b0000;
    //IEC0bits.T1IE = 0b1; //start timeout timer
    I2C1CONbits.SEN = 0b1; //bus start
    while(I2C1CONbits.SEN) {}
    write((addr << 1) | 0x01); //read address
}
uint8_t read(bool do_you_ACK) {
    I2C1CONbits.RCEN = 0b1; //read
    while(I2C1CONbits.RCEN) {}
    I2C1CONbits.ACKDT = !do_you_ACK;
//    while(PORTBbits.RB8) { //bus stuck, perform percussive maintenance
//        I2C1CONbits.I2CEN = 0b0;
//        LATBbits.LATB8 = 0;
//        LATBbits.LATB8 = 1;
//    }
//    I2C1CONbits.I2CEN = 0b1;
//    if(I2C1STATbits.BCL) {//bus collision
//        I2C1STATbits.BCL = 0b0;
//        I2C1CONbits.I2CEN = 0b0;
//        LATBbits.LATB8 = 0;
//        LATBbits.LATB8 = 1;
//        I2C1CONbits.I2CEN = 0b1;
//    } 
    I2C1CONbits.ACKEN = 0b1; //say we read it
    while(I2C1CONbits.ACKEN) {
        if(I2C1STATbits.BCL) { //bus is stuck; perform percussive maintenance
            I2C1CONbits.I2CEN = 0b0;
            TRISBbits.TRISB8 = 0b0;
            LATBbits.LATB8 = 0b0;
            Nop();
            Nop();
            LATBbits.LATB8 = 0b1;
            TRISBbits.TRISB8 = 0b1;
            I2C1CONbits.I2CEN = 0b1;
        }
    }
    return I2C1RCV;
}