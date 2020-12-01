/*
 * LSM6DS3_IMU.C
 * Slave address 0b1101011  (0xD6, 0xD7)Left Justified (0x6B)Right Justified
 * Created: 6/25/2019 19:42:12
 *  Author: Reed
 */ 

#include "LSM6DS3_IMU.h" //Always include the .h file in the .c file


/* The LSM6DS3 has over 100 internal registers that are 8 bits in size.
These registers can be read using IMU_readByte(). The value passed is the register that will be read. */
uint8_t IMU_readByte(uint8_t cmd){ 
	i2c_start(LSM6DS3_ADDR<<1);			//I2C Write start. Address is left shifted.
	i2c_write(cmd);						//Write the register that we want to read.
	i2c_stop();							//I2C stop function.
	
	_delay_us(5);						//Wait some time. Not needed, but helps to keep the data stream readable on a scope.
	
	i2c_start((LSM6DS3_ADDR<<1) | 1);	//I2C Read start. Address is left shifted and or'd with one.
	uint8_t data = i2c_read(1);		//Read a byte, do not acknowledge.  
	i2c_stop();							//I2C stop function. 
	return data;						//Return the byte read.
}

/* The LSM6DS3 has over 100 internal registers that are 8 bits in size.
These registers can be written using IMU_writeByte(). The desired register is cmd. The data is byte*/
void IMU_writeByte(uint8_t cmd, uint8_t byte){
	i2c_start(LSM6DS3_ADDR<<1);			//I2C Write start.
	i2c_write(cmd);						//Write what register to change.
	i2c_write(byte);						//Write the data for that register.
	i2c_stop();							//I2C stop function.
}

void IMU_init(void){
	IMU_writeByte(0x10, 0x4E);			//Configure the Linear Acceleration Control Register (0x10) to 0x43. See data sheet to 0x43 setting details.
	_delay_us(5);						//Wait some time. Not needed, but helps to keep the data stream readable on a scope.
	IMU_writeByte(0x11, 0x4E);			//Configure the Angular Acceleration Control Register (0x11) to 0x43. See data sheet to 0x43 setting details. 
}

int IMU_X_Accel(void){
	//Read registers 0x28, 0x29
	uint8_t upperByte = IMU_readByte(0x29);			//Read the upper half of the signed int
	uint8_t lowerByte = IMU_readByte(0x28);			//Read the lower half of the signed int
	int temp = twoBytes2Int(upperByte, lowerByte);  //Put the two bytes into a signed int
	return temp;
}

int IMU_Y_Accel(void){
	//Read registers 0x2A, 0x2B
	uint8_t upperByte = IMU_readByte(0x2B);			//Read the upper half of the signed int
	uint8_t lowerByte = IMU_readByte(0x2A);			//Read the lower half of the signed int
	int temp = twoBytes2Int(upperByte, lowerByte);  //Put the two bytes into a signed int
	return temp;
}

int IMU_Z_Accel(void){
	//Read registers 0x2C, 0x2D
	uint8_t upperByte = IMU_readByte(0x2D);			//Read the upper half of the signed int
	uint8_t lowerByte = IMU_readByte(0x2C);			//Read the lower half of the signed int
	int temp = twoBytes2Int(upperByte, lowerByte);  //Put the two bytes into a signed int
	return temp;
}

int IMU_X_Pitch(void){
	//Read registers 0x22, 0x23
	uint8_t upperByte = IMU_readByte(0x23);			//Read the upper half of the signed int
	uint8_t lowerByte = IMU_readByte(0x22);			//Read the lower half of the signed int
	int temp = twoBytes2Int(upperByte, lowerByte);  //Put the two bytes into a signed int
	return temp;
}

int IMU_Y_Roll(void){
	//Read registers 0x24, 0x25
	uint8_t upperByte = IMU_readByte(0x25);			//Read the upper half of the signed int
	uint8_t lowerByte = IMU_readByte(0x24);			//Read the lower half of the signed int
	int temp = twoBytes2Int(upperByte, lowerByte);  //Put the two bytes into a signed int
	return temp;
}

int IMU_Z_Yaw(void){
	//Read registers 0x26, 0x27
	uint8_t upperByte = IMU_readByte(0x27);			//Read the upper half of the signed int
	uint8_t lowerByte = IMU_readByte(0x26);			//Read the lower half of the signed int
	int temp = twoBytes2Int(upperByte, lowerByte);  //Put the two bytes into a signed int
	return temp;
}

int IMU_temperature(void){
	//Read registers 0x20, 0x21
	uint8_t upperByte = IMU_readByte(0x21);			//Read the upper half of the signed int
	uint8_t lowerByte = IMU_readByte(0x20);			//Read the lower half of the signed int
	int temp = twoBytes2Int(upperByte, lowerByte);  //Put the two bytes into a signed int
	return temp;
}

int twoBytes2Int (uint8_t upper, uint8_t lower){
	int temp;							//Create a signed int.
	int* ptrTemp = &temp;				//Create a pointer to the signed int. Pointers are needed to save a data type uint8_t to a int.
	*ptrTemp = ((upper<<8) | lower);	//Shift the upper byte 8 places to the left, bitwise OR with lower byte. Save to deferenced pointer. 
	return temp;						//signed int temp now holds ((upper<<8) | lower).
}