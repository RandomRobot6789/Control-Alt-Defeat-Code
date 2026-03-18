/*
 * File:   driving2.c
 * Author: williammulberry
 *
 * Created on February 18, 2026, 10:03 AM
 */


#include "xc.h"
#pragma config FNOSC = FRCDIV

#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

enum drivemode {forward, left, right, decider};
int steps = 0;
int steps_needed = 139;
uint16_t threshold = 165;
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    
    steps ++;
    _OC1IF = 0;
    
}

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

//If at any time a sensor appears to be reading 0xFFFF, it's not at the address you think it is and it's not getting read at all
//because I'm a lazy bum and didn't check for slave ACKs.

#include "settings.h"
#include "VL53L0X.h"
#include "i2c_driver.h"


#pragma config ICS = PGx3 //enable debug


#define EXPANDER_ADDR 0b0111000
#define LED_ARR_ADDR 0b0111001

#define ADDRESS_DEFAULT 0b0101001

static uint8_t LATC = 0x00; //IO expander
static uint8_t LATD = 0x00; //LED bank

uint16_t TOF_l;
uint16_t TOF_m;
uint16_t TOF_r;


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



void adjust_differential(uint16_t left) {
    static uint16_t old_left = 0;
    int base_oc2rs = 1249;
    OC2RS =  base_oc2rs + ((left - old_left)>>2);//right wheel period;
    old_left = left;
}

enum drivemode mode = forward;

void canyon(uint16_t vTOF_L, uint16_t vTOF_M, uint16_t vTOF_R){
    switch(mode){
        case forward://left and right steppers same direction, same speed
            _LATB2 = 0;
            _LATB4 = 0;
            if(vTOF_M <= threshold){
                mode = decider;
            }
            break;
        case decider:
            if(vTOF_L < vTOF_R){
                _OC1IE = 1; //enabled
                steps = 0;
                mode = right;
            }
            else{
                _OC1IE = 1;
                steps = 0;
                mode = left;
            }
            break;
        case left:
            _LATB4 = 1;
            _LATB2 = 0;
            if(steps >= steps_needed){
                _OC1IE = 0;
                mode = forward;
            }
            break;
        case right:
            _LATB4 = 0;
            _LATB2 = 1;
            if(steps >= steps_needed){
                _OC1IE = 0;
                mode = forward;
            }
            break;
    }
}


struct sensor_instance sensors[NUM_OF_TOFS];

int main(void) {
    _RCDIV = 0b001;;
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
    _LATA0 = 0;// start with it on
    _TRISB4 = 0;// direction pin
    _LATB4 = 0;
    
    //Stepper 2/right pins
    _TRISB0 = 0; // pwm pin 4
    _LATB0 = 0;
    _TRISB1 = 0;//disable pin
    _LATB1 = 0;
    _TRISB2 = 0;// direction pin
    _LATB2 = 0;
    
    //switch pin - pull down resistor
    //_TRISB7 = 1;//input
            
    //set up pwm 
    OC1CON1bits.OCM = 0b110; //edge aligned pwm
    OC2CON1bits.OCM = 0b110;
    
    OC1RS = 12049; //frequency and duty cycle of each
    OC1R = 6000;
    
    OC2RS = 12049;
    OC2R = 6000;
    
    //interrupt setup
    _OC1IP = 4; // Select the interrupt priority
    _OC1IE = 0; // disable the OC1 interrupt for now
    _OC1IF = 0; // Clear the interrupt flag
    
    init_i2c();
    
    //get expanders in a known state
    write_expander(EXPANDER_ADDR, 0x00);
    write_expander(LED_ARR_ADDR, 0x00); 
    
    struct sensor_instance sensors[NUM_OF_TOFS];
    struct sensor_instance default_address_dummy = construct_sensor_instance();
    for (int i=0; i<NUM_OF_TOFS; i++) {
//        LATC = LATC && (1 << i); 
        LATC = LATC | (1 << (2*i)); //temporary hack, remove with new pcb
        for(int k=0; k<100; k++) {
            write_expander(EXPANDER_ADDR, LATC);
        }
        sensors[i] = construct_sensor_instance();
        uint8_t new_addr = 0x40 + i;
        setAddress(&(sensors[i]), new_addr);
        //Make sure that worked:
        for(int k=0; k<100; k++) {
            writeReg(&(default_address_dummy), I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
        }
        init(&(sensors[i]), true);
        startContinuous(&(sensors[i]), 0);
    }
    

    while(1){
        TOF_m = readRangeContinuousMillimeters(&(sensors[FRONT]));
        TOF_r = readRangeContinuousMillimeters(&(sensors[RIGHT]));
        TOF_l = readRangeContinuousMillimeters(&(sensors[LEFT]));
        //read values from TOF sensors
        //perhaps filter them
        canyon(TOF_l, TOF_m, TOF_r);
        write_expander(LED_ARR_ADDR, (((TOF_m >> 3) & 0xFF)^((TOF_r >> 3) & 0xFF)^((TOF_l >> 3) & 0xFF)) ^ 0xFF);
    }
    return 0;
}
