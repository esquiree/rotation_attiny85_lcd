/*
 * LSM6DS3_IMU.h
 * Slave address 0b1101011  (0xD6, 0xD7)Left Justified (0x6B)Right Justified
 * Created: 6/25/2019 19:42:12
 *  Author: Reed
 */ 

#include <stdint.h>  			//Include standard C data types. This gives us uint8_t.
#ifndef F_CPU					//Define F_CPU to 16MHz if it is undefined.
#define F_CPU 16000000UL
#endif
#include <util/delay.h>			//Include to get _delay_us functions.
#include "USI_I2C_Master.h"				//I2C Read/Write Functions

#define LSM6DS3_ADDR 0x6B		//I2C addresses are 7 bit values. This the right side justified hex value of 0b1101011.

uint8_t IMU_readByte(uint8_t cmd);	//Used to read a byte from IMU registers. 

void IMU_writeByte(uint8_t, uint8_t); //Used to write a byte from IMU registers.  

void IMU_init(void);				//Sets up linear and angular acceleration control registers.

int IMU_X_Accel(void);				//Returns signed X linear acceleration.

int IMU_Y_Accel(void);				//Returns signed Y linear acceleration.

int IMU_Z_Accel(void);				//Returns signed Z linear acceleration.

int IMU_X_Pitch(void);				//Returns signed X angular acceleration.

int IMU_Y_Roll(void);				//Returns signed Y angular acceleration.

int IMU_Z_Yaw(void);				//Returns signed Z angular acceleration.

int IMU_temperature(void);			//Returns signed temperature. See data sheet from units.

int twoBytes2Int(uint8_t, uint8_t);	//Converts two uint8_t into a int data type.