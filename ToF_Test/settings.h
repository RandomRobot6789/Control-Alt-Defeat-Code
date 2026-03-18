/* 
 * File:   settings.h
 * Author: Andrew Jenkins
 * Comments: Build settings
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SETTINGS_H
#define	SETTINGS_H

#define NUM_OF_TOFS 3

#define OSC_OFF LATBbits.LATB7 = 0
#define OSC_ON LATBbits.LATB7 = 1
//#define OSC_I2C_START
//#define OCS_TOF_READ

#endif	/* SETTINGS_H */

