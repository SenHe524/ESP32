#ifndef _INIT_READ_H_
#define _INIT_READ_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"

/***************************L3G4200D**********************************/
int Init_L3G4200D(void);



/***************************HMC5883L**********************************/
int Init_HMC5883L(void);
int HMC5883L_READ(uint8_t *BUF, float *Angle);


/***************************BMP085**********************************/
void Init_BMP085(short *AC_123, unsigned short *AC_456, short *B1_MD);

int BMP085_READ_TEMPERATURE(long *temperature_temp);

int BMP085_READ_PRESSURE(long *pressure_temp);

int BMP085_DATA(long *temperature_temp, long *pressure_temp, short *AC_123, unsigned short *AC_456, short *B1_MD);


/****************************ADXL345**************************************/
int Init_ADXL345(void);

int ADXL345_AUTO_Adjust(void);

void ADXL345_Read(int *X, int *Y, int *Z);

void ADXL345_Read_Average(int *X, int *Y, int *Z, int times);

float ADXL345_Angle(float A_X, float A_Y, float A_Z, int i);


#endif