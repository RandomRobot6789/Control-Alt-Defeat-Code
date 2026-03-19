/*
 * File:   main.c
 * Author: Andrew_Jenkins
 *
 * Created on February 3, 2026, 12:45 PM
 */

//SCL and SDA are pins 12 and 13, respectively
//Recommended values for pull-up resistors for an AVDD of 2.8 V and a 400 kHz I²C
//clock are 1.5 k to 2 kOhms
//from sensor docs

//from MCU docs: 2.2kO typical
//sensor already has a 10kO pullup; it's plainly insufficient

//Minimum is 132 ohms, but that would be operating with no electrical margins
#include "xc.h"
#include "settings.h"
#include "VL53L0X.h"
#include "i2c_driver.h"

#pragma config FNOSC = FRC //8MHz; 4MHz execution clock
#pragma config ICS = PGx3 //enable debug


#define EXPANDER_ADDR 0b0111000
#define LED_ARR_ADDR 0b0111001

static uint8_t LATC = 0x00; //IO expander
static uint8_t LATD = 0x00; //LED bank

uint16_t front;
uint16_t right;
uint16_t left;


void write_expander(uint8_t address, uint8_t byte) {
    start(address);
    write(byte);
    stop();
}

enum which_ToF {
    FRONT,
    RIGHT,
    LEFT,
    SAMPLE_RETURN,
    SPARE
};

int main() {
    ANSA = 0;
    ANSB = 0;
    TRISBbits.TRISB7 = 0;
//    TRISBbits.TRISB8 = 0;
//    TRISBbits.TRISB9 = 0;

    
    init_i2c();
    
    //get expanders in a known state
    write_expander(EXPANDER_ADDR, 0x00);
    write_expander(LED_ARR_ADDR, 0x00); 
    
    struct sensor_instance sensors[NUM_OF_TOFS];
    for (int i=0; i<NUM_OF_TOFS; i++) {
//        LATC = LATC && (1 << i); 
        LATC = LATC | (1 << (2*i)); //temporary hack, remove with new pcb
        write_expander(EXPANDER_ADDR, LATC);
        sensors[i] = construct_sensor_instance();
        init(&(sensors[i]), true);
        startContinuous(&(sensors[i]), 0);
        setAddress(&(sensors[i]), 0x40 + i);
    }
    

    
    while(true) {
        front = readRangeContinuousMillimeters(&(sensors[FRONT]));
        right = readRangeContinuousMillimeters(&(sensors[RIGHT]));
        left = readRangeContinuousMillimeters(&(sensors[LEFT]));
//        if(timeoutOccurred(&sensor_front)) {
//             init(&sensor_front, true);
//             startContinuous(&sensor_front, 0);
//        }
        //write_expander(LED_ARR_ADDR, ((front >> 3) & 0xFF) ^ 0xFF);
        write_expander(LED_ARR_ADDR, (((front >> 3) & 0xFF)^((right >> 3) & 0xFF)^((left >> 3) & 0xFF)) ^ 0xFF);
        
    }
    return 0;
}