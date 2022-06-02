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

int Init_L3G4200D(float *BUF_MARK);
void L3G4200D_FIFO_READ(float *BUF_L3G4200D, float *BUF_MARK);
void L3G4200D_READ(float *BUF_L3G4200D);
void L3G4200D_READ_AVERAGE(float *BUF_L3G4200D, float *BUF_MARK, int times);

/****************************ADXL345**************************************/
int Init_ADXL345(void);
int ADXL345_AUTO_Adjust(void);
void ADXL345_Read(int *X, int *Y, int *Z);
void ADXL345_Read_Average(int *X, int *Y, int *Z, int times);
float ADXL345_Angle(float A_X, float A_Y, float A_Z, int i);

/***************************HMC5883L**********************************/
int Init_HMC5883L(void);
// void HMC5883L_RAW_READ(uint8_t *BUF, float *mag_XYZ);
void HMC5883L_RAW_READ(uint8_t *BUF, float *Angle, float *mag_XYZ);
// void HMC5883L_SELFTEST(float *Offset, float *K_XYZ);
// void HMC5883L_READ(uint8_t *BUF, float *Angle, float *mag_XYZ, float *Offset, float *K_XYZ);

/***************************BMP085**********************************/
void Init_BMP085(short *AC_123, unsigned short *AC_456, short *B1_MD);
int BMP085_READ_TEMPERATURE(long *temperature_temp);
int BMP085_READ_PRESSURE(long *pressure_temp);
int BMP085_DATA_CALCULATE(long *temperature_temp, long *pressure_temp, short *AC_123, unsigned short *AC_456, short *B1_MD);

#endif