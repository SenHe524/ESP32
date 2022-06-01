#ifndef _REGISTER_H_
#define _REGISTER_H_



/* -------L3G4200 Register names ------- */
#define	L3G4200_Addr            0xD2	//陀螺仪传感器器件地址
#define L3G4200_DEVID           0x0F
#define L3G4200_CTRL_REG1       0x20
#define L3G4200_CTRL_REG2       0x21
#define L3G4200_CTRL_REG3       0x22
#define L3G4200_CTRL_REG4       0x23
#define L3G4200_CTRL_REG5       0x24
#define L3G4200_OUT_X_L         0x28
#define L3G4200_OUT_X_H         0x29
#define L3G4200_OUT_Y_L         0x2A
#define L3G4200_OUT_Y_H         0x2B
#define L3G4200_OUT_Z_L         0x2C
#define L3G4200_OUT_Z_H         0x2D
#define L3G4200_REFERENCE       0x25
#define L3G4200_TEMP            0x26
#define L3G4200_STATUS          0x27
/* -------HMC5883L Register names ------- */
#define	HMC5883L_Addr   0x3C	//磁场传感器器件地址
#define	HMC5883L_Configuration_Register_A       0x00
#define	HMC5883L_Configuration_Register_B       0x01
#define	HMC5883L_Mode_Register      0x02
#define	HMC5883L_Register_XMSB      0x03
#define	HMC5883L_Register_XLSB      0x04
#define	HMC5883L_Register_ZMSB      0x05
#define	HMC5883L_Register_ZLSB      0x06
#define	HMC5883L_Register_YMSB      0x07
#define	HMC5883L_Register_YLSB      0x08
#define	HMC5883L_Status_Register    0x09
#define	HMC5883L_Identification_Register_A      0x10
#define	HMC5883L_Identification_Register_B      0x11
#define	HMC5883L_Identification_Register_C      0x12


/* -------BMP085 Register names ------- */
#define	BMP085_Addr     0xEE	//气压传感器器件地址
#define BMP085_AC1      0xAA
#define BMP085_AC2      0xAC
#define BMP085_AC3      0xAE
#define BMP085_AC4      0xB0
#define BMP085_AC5      0xB2
#define BMP085_AC6      0xB4
#define BMP085_B1       0xB6
#define BMP085_B2       0xB8
#define BMP085_MB       0xBA
#define BMP085_MC       0xBC
#define BMP085_MD       0xBE
#define BMP085_DATA_ADDR        0xF4
#define BMP085_DATA_MSB         0xF6
#define BMP085_DATA_LSB         0xF7
#define BMP085_TEMPERATURE      0x2E
#define BMP085_PRESSURE         0x34

/* -------ADXL345 Register names ------- */
#define	ADXL345_Addr          0xA6	//加速度传感器器件地址
#define ADXL345_DEVID           0x00
#define ADXL345_RESERVED1       0x01
#define ADXL345_THRESH_TAP      0x1d
#define ADXL345_OFSX            0x1e
#define ADXL345_OFSY            0x1f
#define ADXL345_OFSZ            0x20
#define ADXL345_DUR             0x21
#define ADXL345_LATENT          0x22
#define ADXL345_WINDOW          0x23
#define ADXL345_THRESH_ACT      0x24
#define ADXL345_THRESH_INACT    0x25
#define ADXL345_TIME_INACT      0x26
#define ADXL345_ACT_INACT_CTL   0x27
#define ADXL345_THRESH_FF       0x28
#define ADXL345_TIME_FF         0x29
#define ADXL345_TAP_AXES        0x2a
#define ADXL345_ACT_TAP_STATUS  0x2b
#define ADXL345_BW_RATE         0x2c
#define ADXL345_POWER_CTL       0x2d
#define ADXL345_INT_ENABLE      0x2e
#define ADXL345_INT_MAP         0x2f
#define ADXL345_INT_SOURCE      0x30
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_DATAX0          0x32
#define ADXL345_DATAX1          0x33
#define ADXL345_DATAY0          0x34
#define ADXL345_DATAY1          0x35
#define ADXL345_DATAZ0          0x36
#define ADXL345_DATAZ1          0x37
#define ADXL345_FIFO_CTL        0x38
#define ADXL345_FIFO_STATUS     0x39

#endif