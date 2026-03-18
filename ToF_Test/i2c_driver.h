/* 
 * File:        i2c_driver.h
 * Author:      Andrew_Jenkins
 * Comments:    This replaces the <Wire.h> header and the TwoWire class for code ported from Arduino libraries
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef I2C_DRIVER_H
#define	I2C_DRIVER_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdint.h>
#include <stdbool.h>

//code to replace TwoWire and Wire.h:
void init_i2c();

void start(uint8_t addr);
void write(uint8_t byte_to_write);
void stop();

void requestFrom(uint8_t addr, uint8_t num_bytes);
uint8_t read(bool do_you_ACK);

#endif	/* I2C_DRIVER_H */

