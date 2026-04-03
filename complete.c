/*
 * File:   newmainXC16.c
 * Author: williammulberry
 *
 * Created on March 19, 2026, 12:44 PM
 */


#include "xc.h"
#include "settings.h"
#include "VL53L0X.h"
#include "i2c_driver.h"
#pragma config ICS = PGx3

#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG



void config_pins_for_ad(void)
{
    // Put all relevant registers into a known state
    TRISA = 0xFFFF;
    TRISB = 0xFFFF;
    ANSA = 0;
    ANSB = 0;
    //REMOVE ALL PINS EXCEPT FOR THE ONES FOR THE QRD
    // Configure the tri state adapter for input for all the analog pins
    TRISAbits.TRISA0 = 1; // Configure pin 2 for input SAMPLE
    TRISAbits.TRISA1 = 1; // Configure pin 3 for input MID QRD
    //TRISBbits.TRISB0 = 1; // Configure pin 4 for input
    //TRISBbits.TRISB1 = 1; // Configure pin 5 for input   
    //TRISBbits.TRISB2 = 1; // Configure pin 6 for input
    TRISAbits.TRISA2 = 1; // Configure pin 7 for input LEFT SENSOR
    TRISAbits.TRISA3 = 1; // Configure pin 8 for input rIGHT SENSOR
    //TRISBbits.TRISB4 = 1; // Configure pin 9 for input 
    //TRISBbits.TRISB12 = 1; // Configure pin 15 for input   
    //TRISBbits.TRISB13 = 1; // Configure pin 16 for input
    //TRISBbits.TRISB14 = 1; // Configure pin 17 for input
    //TRISBbits.TRISB15 = 1; // Configure pin 18 for input

    // Configure all the analog pins for analog (as opposed to digital)
    ANSAbits.ANSA0 = 1; // Configure pin 2 for analog SAMPLE QRD
    ANSAbits.ANSA1 = 1; // Configure pin 3 for analog MID QRD
    //ANSBbits.ANSB0 = 1; // Configure pin 4 for analog
    //ANSBbits.ANSB1 = 1; // Configure pin 5 for analog   
    //ANSBbits.ANSB2 = 1; // Configure pin 6 for analog
    ANSAbits.ANSA2 = 1; // Configure pin 7 for analOG LEFT SENSOR
    ANSAbits.ANSA3 = 1; // Configure pin 8 for analog RIGHT SENSOR
    //ANSBbits.ANSB4 = 1; // Configure pin 9 for analog 
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
    _SMPI = 3;    // AD1CON2<6:2> -- Every 9th conversion sent (number of channels sampled -1)
                  // to buffer (if sampling 10 channels)
    _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only

    //_CN2PDE = 1;

    // AD1CON3 register
    _ADRC = 0;    // AD1CON3<15> -- Use system clock
    _SAMC = 0b00001;    // AD1CON3<12:8> -- Auto sample every A/D
                  // period TAD
    _ADCS = 0b00000010;    // AD1CON3<7:0> -- TAD needs to be at least 750 ns. Thus, _ADCS = 0b00000010 will give us the fastest AD clock given a 4 MHz system clock.

    // AD1CSS registers
    AD1CSSL = 0b0110000000000011; // choose A2D channels you'd like to scan here.
    //this is doing pins AN 0,13,14,15 which is pins 7,8,9
    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}

#pragma config FNOSC = FRCDIV

double leftval = 0; //pin 13
double midval = 0; //read from pin 14
double rightval = 0; // pin 15

double goal = 2.5;//These are the midline values for each sensor mid sensor
double goal_l = 2.75; //left sensor
double goal_r = 2.5; // right sensor

double freq_right = 0; //the pwm frequencies at which the motor will drive
double freq_left = 0;

int i = 0;


double pwm_output = 0;// control signal in line following

double interval = .01;//if we use a time interval

double kd = 0;//proportional and derivative gains
double kp = 1;

double fcy = 2000000;

double max_freq = 351;//This is the fastest we want the steppers to move
double min_freq = 85;//slowest
double std_freq = 218;


enum drivemode {forward, left, right, decider};
int steps = 0;

uint16_t threshold = 165;



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






void line_follower(double leftval, double midval, double rightval){
        _LATB7 = 0;

        
        double error = midval - goal;
        
        static double olderror = 0;
        static double alpha = 0;
        error = (alpha*olderror) + ((1-alpha)*error);
        double error_derivative = error - olderror;
        olderror = error;

        pwm_output = kp * error - kd * error_derivative;
        freq_right = std_freq * (1+pwm_output);
        freq_left = std_freq * (1 - pwm_output);
        if(freq_right <= min_freq){ //set max and min speeds for motors
            OC2RS = fcy / min_freq;
            _LATB7 = 1;
        }
        else if(freq_right >= max_freq){
            OC2RS = fcy / max_freq;
            _LATB7 = 1;
        }
        else{
            OC2RS = fcy / freq_right;
        }

        if(freq_left <= min_freq){
            OC3RS = fcy / min_freq;
            _LATB7 = 1;
        }
        else if(freq_left >= max_freq){
            OC3RS = fcy / max_freq;
            _LATB7 = 1;
        }
        else{
            OC3RS = fcy / freq_left;
        }
        OC2R = .5*OC2RS;
        OC3R = .5*OC3RS;


        if(leftval < goal_l){
            OC3RS= fcy/min_freq;
            OC2RS = fcy/max_freq;
            OC3R = 0;
            OC2R = .5*OC2RS;
            _LATB7 = 1;
        }
        if(rightval < goal_r){
            OC3RS= fcy/max_freq;
            OC2RS = fcy/min_freq;
            OC3R = .5*OC3RS;
            OC2R = 0;
            _LATB7 = 1;
        }
}
void straight(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 6000;
    OC3R = 6000;
    _LATB4 = 0;
    _LATA4 = 0;
}
void left_center(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 6000;
    OC3R = 6000;
    _LATB4 = 1;
    _LATA4 = 0;
}
void right_center(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 6000;
    OC3R = 6000;
    _LATB4 = 0;
    _LATA4 = 1;
}
void left_pivot(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 6000;
    OC3R = 0;
    _LATB4 = 0;
    _LATA4 = 0;
}
void right_pivot(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 0;
    OC3R = 6000;
    _LATB4 = 0;
    _LATA4 = 0;
}
void reverse(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 6000;
    OC3R = 6000;
    _LATB4 = 1;
    _LATA4 = 1;
}
void stop(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 0;
    OC3R = 0;
    _LATB4 = 0;
    _LATA4 = 0;
}

void findline1(){
    static int state = 1;
    swtich(state){
        case 1:
            forward();
            if(leftval < 2.8 && rightval <2.8){
                steps_needed = 270;
                state = 2;
            } 
            break;
        case 2:
            left_pivot();
            if(steps >= steps_needed){
                OC3IE = 0;
                steps = 0;
                state = 1;
                ss = line;
            }
            break;
            
    }
}

void samp_collect(int turn_steps, int move_steps){
    static int state = 1;
    switch(state){
        case 1:
            _OC3IE = 1;
            steps = 0;
            right_center();
            state = 2;
            break;
        case 2:
            if(steps >= turn_steps){
                steps = 0;
                forward();
                state = 3;
            }
            break;
        case 3:
            if(steps >= move_steps){
                steps = 0;
                stop();
                state = 4;
                i = 0;
            }
            break;
        case 4:
            i++;
            if(i >= 25){
                steps = 0;
                reverse();
                state = 5;
                i = 0;
            }
            break;
        case 5:
            if(steps >= move_steps){
                steps = 0;
                left_center();
                state = 6;
            }
            break;
        case 6:
            if(steps >= turn_steps){
                _OC3IE = 0;
                steps = 0;
                ss = line;
            }
            break;   
    }
}

void Solenoid_On(int pin){
    LATC = LATC | (1 << pin);
    write_expander(EXPANDER_ADDR, LATC);
}

void Solenoid_Off(int pin){
    LATC = LATC & (~(1 << pin));
    write_expander(EXPANDER_ADDR, LATC);
}




void samp_return(int turn_steps, int move_steps, double qrd_val){
    static int state = 1;
    static int white = 1;
    switch(state){
        case 1:
            _OC3IE = 1;
            steps = 0;
            if(qrd_val < 2.5){
                left_center();
                white = 1;
            }
            else{
                right_center();  
                white = 0;
            }
            state = 2;
            break;
        case 2:
            if(steps >= turn_steps){
                steps = 0;
                forward();
                state = 3;
            }
            break;
        case 3:
            if(steps >= move_steps){
                steps = 0;
                stop();
                if(white ==1){
                    Solenoid_On(5);
                }
                else{
                    Solenoid_On(6);
                }
                state = 4;
                i = 0;
            }
            break;
        case 4:
            i++;
            if(i >= 25){
                steps = 0;
                Solenoid_Off(5);
                Solenoid_Off(6);
                reverse();
                state = 5;
                i = 0;
            }
            break;
        case 5:
            if(steps >= move_steps){
                steps = 0;
                left_center();
                state = 6;
            }
            break;
        case 6:
            if(steps >= turn_steps){
                _OC3IE = 0;
                steps = 0;
                ss = line;
            }
            break;   
    }
}

void canyon(uint16_t vTOF_L, uint16_t vTOF_M, uint16_t vTOF_R, double qrdval, int turn_steps, int pivot_steps){
    switch(state){
        case 0:
            forward();
            state = 1;
        case 1://left and right steppers same direction, same speed
            if(vTOF_M <= threshold){
                state = 2;
            }
            else if(qrdval <= 2.4){
                if(vTOF_L < vTOF_R){
                    right_pivot();
                }
                else{
                    left_pivot()
                }
                steps = 0;
                _OC3IE = 1;
                state = 4;
            }
            break;
        case 2:
            if(vTOF_L < vTOF_R){
                _OC3IE = 1; //enabled
                right_center();
                steps = 0;
                state = 3;
            }
            else{
                _OC3IE = 1;
                left_center();
                steps = 0;
                state = 3;
            }
            break;
        case 3:
            if(steps >= turn_steps){
                _OC3IE = 0;
                forward();
                state = 1;
            }
            break;
        case 4:
            if(steps >= pivot_steps){
                steps = 0;
                _OC3IE = 0;
                ss = line;
            }
            
    }
}

void data_trans(int first_steps, int second_steps, int_third_steps){
    switch(state){
        case 0:
            _OC3IE = 1;
            forward();
            steps = 0;
            state = 1;
            break;
        case 1:
            if(steps >= first_steps){
                steps = 0;
                left_pivot();
                state = 2;
            }
        case 2:
            line_following(lefval, midval, rightval);
            if(
    }
}


void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void){
    
    steps ++;
    _OC3IF = 0;
    
}

struct sensor_instance sensors[NUM_OF_TOFS];

enum superstate {start, line, collection, canyon, samp_return, data_trans} 

enum superstate ss = start;

int main(void) {

    config_pins_for_ad();
    config_ad();
    
    _RCDIV = 0b001;
    OC3CON1 = 0; // sets all settings to 0 for 1 and 2 ALL for pin 14/RA6
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111; // system clock chosen
    OC3CON2bits.SYNCSEL = 0b11111; // output compare module
    OC3CON2bits.OCTRIG = 0; // he says just do this one
    
    OC2CON1 = 0; // same thing on pin 4/RB0
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111; //
    OC2CON2bits.SYNCSEL = 0b11111; //
    OC2CON2bits.OCTRIG = 0; //
    
    //Stepper 1/left pins
    _TRISB1 = 0; //pwm pin 14
    _LATB1 = 0;
    _TRISB4 = 0;// direction pin
    _LATB4 = 0;
    
    //Stepper 2/right pins
    _TRISB0 = 0; // pwm pin 4
    _LATB0 = 0;
    _TRISA4 = 0;// direction pin
    _LATA4 = 0;
    
    //switch pin - pull down resistor
    //_TRISB7 = 1;//input
            
    //set up pwm 
    OC3CON1bits.OCM = 0b110; //edge aligned pwm
    OC2CON1bits.OCM = 0b110;
    
    OC3RS = 12049; //frequency and duty cycle of each
    OC3R = 6000;
    
    OC2RS = 12049;
    OC2R = 6000;
    
    //interrupt setup
    _OC3IP = 4; // Select the interrupt priority
    _OC3IE = 0; // disable the OC3 interrupt for now
    _OC3IF = 0; // Clear the interrupt flag
    
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
        switch(ss){
            case start:
                findline1();
                break;
            case line:
                TOF_samp = readRangeContinuousMillimeters(&(sensors[SAMPLE_RETURN]));
                TOF_r = readRangeContinuousMillimeters(&(sensors[RIGHT]));
                TOF_l = readRangeContinuousMillimeters(&(sensors[LEFT]));
                
                leftval = (double)ADC1BUF1/4095*3.3;//collect voltages from QRD's
                midval = (double)ADC1BUF13/4095*3.3;
                rightval = (double)ADC1BUF14/4095*3.3;
                linefollower(leftval, midval, rightval);
                if(_RB12 == 1 && ball == 0){
                    ss = collection;
                }
                if((ball == 1 && TOF_SAMP < threshold) && (TOF_R > threshold)){
                    ss = samp_return;
                }
                if(TOF_l < threshold && TOF_r < threshold){
                    ss = canyon;
                }
                if(leftval < 2.6 && midval < 2.9){
                    ss = data_trans;
                }
                break;
            case collection:
                samp_collect(200,400);
                break;
            case samp_return:
                sampval = (double)ADC1BUF0/4095*3.3;
                samp_return(140, 250, sampval);
                break;
            case canyon:
                TOF_m = readRangeContinuousMillimeters(&(sensors[FRONT]));
                TOF_r = readRangeContinuousMillimeters(&(sensors[RIGHT]));
                TOF_l = readRangeContinuousMillimeters(&(sensors[LEFT]));
                leftval = (double)ADC1BUF1/4095*3.3;
                canyon(TOF_l, TOF_m, TOF_r, leftval, 140, 250);
            case data_trans:
                
                
                
                
                

        }
    }
    return 0;
}
