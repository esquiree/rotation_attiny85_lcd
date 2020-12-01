/*
 * Attiny85_BitBang.cpp
 *
 * Created: 4/6/2020 21:55:32
 * Author : Reed
 */ 
#include <stdio.h>
#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#include "SSD1306.h"
#include "LSM6DS3_IMU.h"
#include "USI_I2C_Master.h"
#include "SSD1306.c"
#include "LSM6DS3_IMU.c"
#include "USI_I2C_Master.c"
#include <math.h>
#define PIN_BUTTON PB1
#define PIN_RED_LED PB4
#define PIN_GREEN_LED PB3

int pitch_X, roll_Y, yaw_Z, accelr_X, accelr_Y, accelr_Z; //Declare memory for IMU data as global variables.
uint8_t display_state;
double vector_A[] = {15,0,0}; //X,Y,Z
double vector_A1[] = {0,15,0}; //X,Y,Z
double vector_A2[] = {0,0,15}; //X,Y,Z
double vector_A3[] = {0,0,0}; //X,Y,Z
double vector_A4[] = {15,15,15}; //X,Y,Z
double vector_B[3]; //X,Y,Z
double vector_C[3]; //X,Y,Z
double vector_D[3]; //X,Y,Z
double rotationMatrix[] = {0, 0, 0}; //From 0 to 2Pi //X,Y,Z


void update_display(); //Declare function to update LCD.

int main(void)
{
    /* Replace with your application code */
	i2c_init(); //Init I2C Library. THIS MUST BE DONE BEFORE OLED AND IMU INIT
	OLED_Init();  //Init LCD
	OLED_Clear(); //Clear LCD
	IMU_init();
	
	
	
	PORTB = (1<<PIN_BUTTON); //PB1 will be used as an input.
	
    while (1) 
    {
		accelr_X = IMU_X_Accel();  //Linear Acceleration
		accelr_Y = IMU_Y_Accel();
		accelr_Z = IMU_Z_Accel();
		
		//pitch_X = IMU_X_Pitch();  //Angular Acceleration
		//roll_Y = IMU_Y_Roll();
		//yaw_Z = IMU_Z_Yaw();
		
		//Perform rotation of angle rotationMatrix[0] on vector_A. Save to vector_B
		vector_B[0] = vector_A4[0];
		vector_B[1] = vector_A4[1] * cos(rotationMatrix[0]) + vector_A4[2] * sin(rotationMatrix[0]);
		vector_B[2] = vector_A4[2] * cos(rotationMatrix[0]) - vector_A4[1] * sin(rotationMatrix[0]);
		
		//Perform rotation of angle rotationMatrix[1] on vector_B. Save to vector_C
		vector_C[0] = vector_B[0] * cos(rotationMatrix[1]) + vector_B[2] * sin(rotationMatrix[1]);
		vector_C[1] = vector_B[1];
		vector_C[2] = vector_B[2] * cos(rotationMatrix[1]) - vector_B[0] * sin(rotationMatrix[1]);
		
		//Perform rotation of angle rotationMatrix[3] on vector_C. Save to vector_D
		vector_D[0] = vector_C[0] * cos(rotationMatrix[2]) + vector_C[1] * sin(rotationMatrix[2]);
		vector_D[1] = vector_C[1] * cos(rotationMatrix[2]) - vector_C[0] * sin(rotationMatrix[2]);
		vector_D[2] = vector_C[2];
		OLED_DisplayPixel_SET(uint8_t(vector_D[0]+64),uint8_t(vector_D[1]+32));
		
		//Perform rotation of angle rotationMatrix[0] on vector_A. Save to vector_B
		vector_B[0] = vector_A3[0];
		vector_B[1] = vector_A3[1] * cos(rotationMatrix[0]) + vector_A3[2] * sin(rotationMatrix[0]);
		vector_B[2] = vector_A3[2] * cos(rotationMatrix[0]) - vector_A3[1] * sin(rotationMatrix[0]);
		
		//Perform rotation of angle rotationMatrix[1] on vector_B. Save to vector_C
		vector_C[0] = vector_B[0] * cos(rotationMatrix[1]) + vector_B[2] * sin(rotationMatrix[1]);
		vector_C[1] = vector_B[1];
		vector_C[2] = vector_B[2] * cos(rotationMatrix[1]) - vector_B[0] * sin(rotationMatrix[1]);
		
		//Perform rotation of angle rotationMatrix[3] on vector_C. Save to vector_D
		vector_D[0] = vector_C[0] * cos(rotationMatrix[2]) + vector_C[1] * sin(rotationMatrix[2]);
		vector_D[1] = vector_C[1] * cos(rotationMatrix[2]) - vector_C[0] * sin(rotationMatrix[2]);
		vector_D[2] = vector_C[2];
		OLED_DisplayPixel_SET(uint8_t(vector_D[0]+64),uint8_t(vector_D[1]+32));
		
		//Perform rotation of angle rotationMatrix[0] on vector_A. Save to vector_B
		vector_B[0] = vector_A2[0];
		vector_B[1] = vector_A2[1] * cos(rotationMatrix[0]) + vector_A2[2] * sin(rotationMatrix[0]);
		vector_B[2] = vector_A2[2] * cos(rotationMatrix[0]) - vector_A2[1] * sin(rotationMatrix[0]);
		
		//Perform rotation of angle rotationMatrix[1] on vector_B. Save to vector_C
		vector_C[0] = vector_B[0] * cos(rotationMatrix[1]) + vector_B[2] * sin(rotationMatrix[1]);
		vector_C[1] = vector_B[1];
		vector_C[2] = vector_B[2] * cos(rotationMatrix[1]) - vector_B[0] * sin(rotationMatrix[1]);
		
		//Perform rotation of angle rotationMatrix[3] on vector_C. Save to vector_D
		vector_D[0] = vector_C[0] * cos(rotationMatrix[2]) + vector_C[1] * sin(rotationMatrix[2]);
		vector_D[1] = vector_C[1] * cos(rotationMatrix[2]) - vector_C[0] * sin(rotationMatrix[2]);
		vector_D[2] = vector_C[2];
		OLED_DisplayPixel_SET(uint8_t(vector_D[0]+64),uint8_t(vector_D[1]+32));
			
		//Perform rotation of angle rotationMatrix[0] on vector_A. Save to vector_B
		vector_B[0] = vector_A1[0];
		vector_B[1] = vector_A1[1] * cos(rotationMatrix[0]) + vector_A1[2] * sin(rotationMatrix[0]);
		vector_B[2] = vector_A1[2] * cos(rotationMatrix[0]) - vector_A1[1] * sin(rotationMatrix[0]);
		
		//Perform rotation of angle rotationMatrix[1] on vector_B. Save to vector_C
		vector_C[0] = vector_B[0] * cos(rotationMatrix[1]) + vector_B[2] * sin(rotationMatrix[1]);
		vector_C[1] = vector_B[1];
		vector_C[2] = vector_B[2] * cos(rotationMatrix[1]) - vector_B[0] * sin(rotationMatrix[1]);
		
		//Perform rotation of angle rotationMatrix[3] on vector_C. Save to vector_D
		vector_D[0] = vector_C[0] * cos(rotationMatrix[2]) + vector_C[1] * sin(rotationMatrix[2]);
		vector_D[1] = vector_C[1] * cos(rotationMatrix[2]) - vector_C[0] * sin(rotationMatrix[2]);
		vector_D[2] = vector_C[2];
		OLED_DisplayPixel_SET(uint8_t(vector_D[0]+64),uint8_t(vector_D[1]+32));
		
		//Perform rotation of angle rotationMatrix[0] on vector_A. Save to vector_B
		vector_B[0] = vector_A[0];
		vector_B[1] = vector_A[1] * cos(rotationMatrix[0]) + vector_A[2] * sin(rotationMatrix[0]);
		vector_B[2] = vector_A[2] * cos(rotationMatrix[0]) - vector_A[1] * sin(rotationMatrix[0]);
		
		//Perform rotation of angle rotationMatrix[1] on vector_B. Save to vector_C
		vector_C[0] = vector_B[0] * cos(rotationMatrix[1]) + vector_B[2] * sin(rotationMatrix[1]);
		vector_C[1] = vector_B[1];
		vector_C[2] = vector_B[2] * cos(rotationMatrix[1]) - vector_B[0] * sin(rotationMatrix[1]);
		
		//Perform rotation of angle rotationMatrix[3] on vector_C. Save to vector_D
		vector_D[0] = vector_C[0] * cos(rotationMatrix[2]) + vector_C[1] * sin(rotationMatrix[2]);
		vector_D[1] = vector_C[1] * cos(rotationMatrix[2]) - vector_C[0] * sin(rotationMatrix[2]);
		vector_D[2] = vector_C[2];
		OLED_DisplayPixel_SET(uint8_t(vector_D[0]+64),uint8_t(vector_D[1]+32));
		
		//OLED_DisplayPixel_SET(uint8_t(vector_D[0]),uint8_t(vector_D[1]));
		update_display();
		_delay_ms(100);
		OLED_Clear();
		rotationMatrix[0] += 0.01;
		rotationMatrix[1] += 0.1;
		rotationMatrix[2] += 0.05;
		//OLED_DisplayPixel_SET(uint8_t(vector_D[0]+64),uint8_t(vector_D[1]+32));
    }
}

void update_display (void){
	OLED_SetCursor(5, 95);
	OLED_DisplayNumber(C_DECIMAL_U8, (uint32_t)accelr_X, 5);
	OLED_SetCursor(5, 3);
	OLED_Printf("(X)");
	OLED_SetCursor(5, 85);
	if (accelr_X>0){
		OLED_Printf("+");
		}else{
		OLED_Printf("-");
	}
		
	OLED_SetCursor(6, 95);
	OLED_DisplayNumber(C_DECIMAL_U8, (uint32_t)accelr_Y, 5);
	OLED_SetCursor(6, 3);
	OLED_Printf("(Y)");
	OLED_SetCursor(6, 85);
	if (accelr_Y>0){
		OLED_Printf("+");
		}else{
		OLED_Printf("-");
	}
		
	OLED_SetCursor(7, 95);
	OLED_DisplayNumber(C_DECIMAL_U8, (uint32_t)accelr_Z, 5);
	OLED_SetCursor(7, 3);
	OLED_Printf("(Z)");
	OLED_SetCursor(7, 85);
	if (accelr_Z>0){
		OLED_Printf("+");
		}else{
		OLED_Printf("-");
	}
}