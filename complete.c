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
    TRISAbits.TRISA3 = 1; // Configure pin 8 for input RIGHT SENSOR
    //TRISBbits.TRISB4 = 1; // Configure pin 9 for input 
    //TRISBbits.TRISB12 = 1; // Configure pin 15 for input   
    TRISBbits.TRISB13 = 1; // Configure pin 16 for input
    TRISBbits.TRISB14 = 1; // Configure pin 17 for input
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
    ANSBbits.ANSB13 = 1; // Configure pin 16 for analog
    ANSBbits.ANSB14 = 1; // Configure pin 17 for analog
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
    _SMPI = 2;    // AD1CON2<6:2> -- Every 9th conversion sent (number of channels sampled -1)
                  // to buffer (if sampling 10 channels)
    _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only

    //_CN2PDE = 1;

    // AD1CON3 register
    _ADRC = 0;    // AD1CON3<15> -- Use system clock
    _SAMC = 0b00001;    // AD1CON3<12:8> -- Auto sample every A/D
                  // period TAD
    _ADCS = 0b00000010;    // AD1CON3<7:0> -- TAD needs to be at least 750 ns. Thus, _ADCS = 0b00000010 will give us the fastest AD clock given a 4 MHz system clock.

    // AD1CSS registers
    AD1CSSL = 0b0110000000000010; // choose A2D channels you'd like to scan here.
    //we have pins 15, 16, 2, 3, 7, 8 which are analog channels 12, 11, 0, 1, 13, 14
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

int ball = 0;
int steps = 0;
int steps_needed;

enum superstate {start_s, line_s, collection_s, canyon_s, samp_return_s, data_trans_s};

enum superstate ss = start_s;

uint16_t threshold = 165;



#define EXPANDER_ADDR 0b0111000
#define LED_ARR_ADDR 0b0111001

#define ADDRESS_DEFAULT 0b0101001

static uint8_t LATC = 0x00; //IO expander
static uint8_t LATD = 0x00; //LED bank

uint16_t TOF_l;
uint16_t TOF_m;
uint16_t TOF_r;
uint16_t TOF_samp;



void write_expander(uint8_t address, uint8_t byte) {
    start(address);
    write(byte);
    stop();
}

void write_state(uint8_t data) { //only lower 3 bits matter
    LATD = (LATD & 0b11111000) | (data & 0b00000111);
    write_expander(LED_ARR_ADDR, LATD);
}

void write_substate(uint8_t data) { //only lower 3 bits matter
    LATD = (LATD & 0b11000111) | ((data << 3) & 0b00111000);
    write_expander(LED_ARR_ADDR, LATD);
}

enum which_ToF {
    FRONT,
    RIGHT,
    LEFT,
    SAMPLE_RETURN,
    SPARE
};


struct sensor_instance sensors[NUM_OF_TOFS];

void init_tofs() {
    write_expander(EXPANDER_ADDR, 0x00);
    
    struct sensor_instance default_address_dummy = construct_sensor_instance();
    for (int i=0; i<NUM_OF_TOFS; i++) {
        LATC = LATC && (1 << i);
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
}

void adjust_differential(uint16_t left) {
    static uint16_t old_left = 0;
    int base_oc2rs = 1249;
    OC2RS =  base_oc2rs + ((left - old_left)>>2);//right wheel period;
    old_left = left;
}






void line_follower(double leftval, double midval, double rightval){

        
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
        }
        else if(freq_right >= max_freq){
            OC2RS = fcy / max_freq;
        }
        else{
            OC2RS = fcy / freq_right;
        }

        if(freq_left <= min_freq){
            OC3RS = fcy / min_freq;
        }
        else if(freq_left >= max_freq){
            OC3RS = fcy / max_freq;
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
        }
        if(rightval < goal_r){
            OC3RS= fcy/max_freq;
            OC2RS = fcy/min_freq;
            OC3R = .5*OC3RS;
            OC2R = 0;
        }
}
void forward(){
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
void stop_robot(){
    OC3RS = 12049;
    OC2RS = 12049;
    OC2R = 0;
    OC3R = 0;
    _LATB4 = 0;
    _LATA4 = 0;
}

void findline1(){
    static int state = 1;
    switch(state){
        case 1:
            forward();
            double left_qrd = (double)ADC1BUF1/4095*3.3;//collect voltages from QRD's
            double right_qrd = (double)ADC1BUF14/4095*3.3;
            if(left_qrd < 2.5 && right_qrd <2.5){
                steps_needed = 270;
                _OC3IE = 1;
                state = 2;
            } 
            break;
        case 2:
            left_pivot();
            if(steps >= steps_needed){
                _OC3IE = 0;
                steps = 0;
                state = 1;
                ss = line_s;
            }
            break;
    }
    write_substate(state);
}

void samp_collect(int turn_steps, int move_steps, int turn_steps2){
    static int state = 1;
    switch(state){
        case 1:
            _OC3IE = 1;
            steps = 0;
            right_pivot();
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
                stop_robot();
                state = 4;
                i = 0;
            }
            break;
        case 4:
            i++;
            if(i >= 1000){
                ball = 1;
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
            if(steps >= turn_steps2){
                _OC3IE = 0;
                steps = 0;
                ss = line_s;
                _LATB4 = 0;
                _LATA4 = 0;
            }
            break;   
    }
    write_substate(state);
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
                stop_robot();
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
                ball = 2;
                ss = line_s;
            }
            break;   
    }
    write_substate(state);
}

void canyon(uint16_t vTOF_L, uint16_t vTOF_M, uint16_t vTOF_R, double qrdval, int turn_steps, int pivot_steps){
    static int state = 0;
    switch(state){
        case 0:
            forward();
            state = 1;
            break;
        case 1://left and right steppers same direction, same speed
            if(vTOF_M <= threshold){
                state = 2;
            }
            else if(qrdval <= 2.4){
                if(vTOF_L < vTOF_R){
                    right_pivot();
                }
                else{
                    left_pivot();
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
                ss = line_s;
            }
            break;    
    }
    write_substate(state);
}

uint16_t data_trans_low_angle = 0;
uint16_t data_trans_high_angle = 0;
bool is_data_trans_measurement_done = false;

void data_trans(int first_steps, int second_steps, int third_steps){
    static int state = 0;
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
                left_center();
                state = 2;
            }
            break;
        case 2:
            if(steps >= second_steps){
                _OC3IE = 0;
                _LATB4 = 0;
                _LATA4 = 0;
                steps = 0;
                state = 3;
            }
            break;
        case 3:
            Nop();
            double left_qrd = (double)ADC1BUF1/4095*3.3;//collect voltages from QRD's
            double mid_qrd = (double)ADC1BUF13/4095*3.3;
            double right_qrd = (double)ADC1BUF14/4095*3.3;
            uint16_t sensor = readRangeContinuousMillimeters(&(sensors[FRONT]));
            line_follower(left_qrd, mid_qrd, right_qrd);
            if(sensor <= 300){
                _OC3IE = 1;
                steps = 0;
                forward();
                state = 4;                  
            }
            break;
        case 4:
            if(steps >= third_steps){
                _OC3IE = 0;
                steps = 0;
                stop_robot();
                state = 5;
                
                //enable OC1 PWM interrupts
                IEC0bits.OC1IE = 0b1;
            }
            break;
        case 5:
            //set up PWM then continually:
            //increment PWM period
            //Measure voltage

            //The algorithm works like this:
            //we have a high and a low threshold
            //as we increase the angle, we measure the first time we cross above the high threshold
            //and when we reach the low threshold again, we look at the last time we crossed below the high threshold
            //find the midpoint of these two angles
            //and aim at that

            //this algorithm lives mostly in the interrupt handler TBH

            if (is_data_trans_measurement_done) {
                IEC0bits.OC1IE = 0b0;
                state = 6;
                OC1R = (data_trans_low_angle + data_trans_high_angle) << 1; //divided by two
                //gotta wait for the motor to get in position before turning on the laser
                //wait 0.1 seconds
                T5CON = 0x8030;
                PR5 = 0xFFFF; 
                TMR5 = 0;
            }
            break;
        case 6:
            for (int i=0; i<5; i++) { //5 times:
                while (TMR5 < 40000) {} //wait 20 ms (blocking code, I know...)
                TMR5 = 0;
            }
            LATBbits.LATB14 = 0b1; //turn on laser
            state = 7; 
            break;
        case 7:
            break;
    }
    write_substate(state);
}


//analog to digital is 12 bit where all 1s is 3.3V
#define DATA_TRANS_LOW_THRESHOLD 0x800
#define DATA_TRANS_HIGH_TRESHOLD 0x600

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    
    uint16_t measurement = ADC1BUF11; //read from that photodiode
    if (measurement > DATA_TRANS_HIGH_TRESHOLD) {
        if (data_trans_low_angle == 0) {
            data_trans_low_angle = OC1R;
        }
        data_trans_high_angle = OC1R;
    }
    else {
        if ((measurement < DATA_TRANS_LOW_THRESHOLD) && (data_trans_low_angle != 0)) {
            is_data_trans_measurement_done = true;
        }
    }

    OC1R += 30; //takes 1.5 seconds to move 90 degrees
    _OC1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void){
    
    steps ++;
    _OC3IF = 0;
    
}

struct sensor_instance sensors[NUM_OF_TOFS];



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

    #define SERVO_PWM_PERIOD 40000 //calculated from 2MHz clock frequency for 50 Hz signal (from datasheet)
    #define SERVO_PWM_MIN_COUNT 1000 //0.5 ms (from datasheet)
    #define SERVO_PWM_MAX_COUNT 5000 //2.5 ms (from datasheet) 
    //but we only go halfway there

    //set up servo PWM:
    TRISAbits.TRISA6 = 0;
    OC1CON1 = 0; // sets all settings to 0 for 1 and 2 ALL for pin 14/RA6
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0b111; // system clock chosen
    OC1CON2bits.SYNCSEL = 0b11111; // output compare module
    OC1CON2bits.OCTRIG = 0;
    OC1CON1bits.OCM = 0b110; //edge aligned pwm; enables the PWM output
    OC1RS = SERVO_PWM_PERIOD;
    OC1R = SERVO_PWM_MIN_COUNT; 
    //this keeps the servo at 0 degrees for most of the course
    TRISBbits.TRISB14 = 0b0; //set up laser pin
    LATBbits.LATB14 = 0b0;
    
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
    
    _OC1IP = 5; // Select the interrupt priority
    _OC1IE = 0; // disable the OC3 interrupt for now
    _OC1IF = 0; // Clear the interrupt flag
    
    init_i2c();
    
    //get expander in a known state
    LATD = 0x00;
    write_expander(LED_ARR_ADDR, LATD); 
    
    init_tofs();
    
    while(1){

        switch(ss){
            case start_s:
                findline1();
                break;
            case line_s:
                TOF_samp = readRangeContinuousMillimeters(&(sensors[SAMPLE_RETURN]));
                TOF_r = readRangeContinuousMillimeters(&(sensors[RIGHT]));
                TOF_l = readRangeContinuousMillimeters(&(sensors[LEFT]));
                
                leftval = (double)ADC1BUF1/4095*3.3;//collect voltages from QRD's
                midval = (double)ADC1BUF13/4095*3.3;
                rightval = (double)ADC1BUF14/4095*3.3;
                line_follower(leftval, midval, rightval);
                if(_RB12 == 0 && ball == 0){
                    ss = collection_s;
                }
                if((ball == 1 && TOF_samp < threshold) && (TOF_r > threshold)){
                    ss = samp_return_s;
                }
                if(TOF_l < threshold && TOF_r < threshold){
                    ss = canyon_s;
                }
                if(leftval < 2.6 && midval < 2.9){
                    ss = data_trans_s;
                }
                break;
            case collection_s:
                samp_collect(280,220,140);
                break;
            case samp_return_s:
                Nop();
                double sampval = (double)ADC1BUF0/4095*3.3;
                samp_return(200, 250, sampval);
                break;
            case canyon_s:
                TOF_m = readRangeContinuousMillimeters(&(sensors[FRONT]));
                TOF_r = readRangeContinuousMillimeters(&(sensors[RIGHT]));
                TOF_l = readRangeContinuousMillimeters(&(sensors[LEFT]));
                leftval = (double)ADC1BUF1/4095*3.3;
                canyon(TOF_l, TOF_m, TOF_r, leftval, 140, 250);
                break;
            case data_trans_s:
                data_trans(55, 140, 50);
                break;
        }
        write_state(ss);
    }
    return 0;
}
