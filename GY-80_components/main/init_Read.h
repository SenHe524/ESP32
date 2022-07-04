#ifndef _INIT_READ_H_
#define _INIT_READ_H_

/***************************L3G4200D**********************************/
// int Init_L3G4200D(void);
void L3G4200D_Read(float *BUF_L3G4200D);
int Init_L3G4200D(float *drift_buf);
void L3G4200D_Read_Average(float *BUF_L3G4200D, int times, float *drift_buf);
// void L3G4200D_Read_Average(float *BUF_L3G4200D, int times);

/****************************ADXL345**************************************/
int Init_ADXL345(void);
int ADXL345_Auto_Adjust(void);
void ADXL345_Read(short *Buf);
void ADXL345_Fifo_Read(float *Buf, int times);
void ADXL345_Read_Average(float *Buf, int times);
float ADXL345_Angle(float *Buf, int i);

/***************************HMC5883L**********************************/
void Init_HMC5883L(void);

// void HMC5883L_Raw_Read(float *Angle, float *mag_XYZ);
void HMC5883L_Raw_Read(float *mag_XYZ);
void HMC5883L_SELFTEST(float *Offset, float *K_XYZ);
void HMC5883L_READ(float *Angle, float *mag_XYZ, float *Offset, float *K_XYZ);

/***************************BMP085**********************************/
void Init_BMP085(short *AC_123, unsigned short *AC_456, short *B1_MD);
void BMP085_Read_Temperature(long *temperature_temp);
void BMP085_Read_Pressure(long *pressure_temp);
void BMP085_Data_Calculate(long *temperature_temp, long *pressure_temp, short *AC_123, unsigned short *AC_456, short *B1_MD);

#endif