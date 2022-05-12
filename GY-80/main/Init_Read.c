#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"
#include "math.h"

#include "I2C.h"
#include "Register.h"

#define OSS 0 //BMP085 使用

/***************************L3G4200D**********************************/
int Init_L3G4200D(void)
{
    if(Single_Read(L3G4200_Addr, L3G4200_DEVID) != 0xD3)        return 0;
    if(!Single_Write(L3G4200_Addr,L3G4200_CTRL_REG1,0x58))      return 0;
    if(!Single_Write(L3G4200_Addr,L3G4200_CTRL_REG2,0x20))      return 0;
    if(!Single_Write(L3G4200_Addr,L3G4200_CTRL_REG3,0x00))      return 0;
    if(!Single_Write(L3G4200_Addr,L3G4200_CTRL_REG4,0x58))      return 0;
    if(!Single_Write(L3G4200_Addr,L3G4200_CTRL_REG5,0x58))      return 0;
    return 1;
}


/***************************HMC5883L**********************************/

int Init_HMC5883L(void)
{
    // if(Single_Read(HMC5883L_Addr, HMC5883L_Identification_Register_A) != 0x48)        return 0;
    if(!Single_Write(HMC5883L_Addr,HMC5883L_Configuration_Register_A,0x58))      return 0;
    if(!Single_Write(HMC5883L_Addr,HMC5883L_Configuration_Register_B,0x60))      return 0;
    if(!Single_Write(HMC5883L_Addr,HMC5883L_Mode_Register,0x00))      return 0;
    return 1;
}
int HMC5883L_READ(uint8_t *BUF, float *Angle)
{
    int x,y,z;
    BUF[0] = Single_Read(HMC5883L_Addr,HMC5883L_Register_XMSB);
    BUF[1] = Single_Read(HMC5883L_Addr,HMC5883L_Register_XLSB);
    BUF[2] = Single_Read(HMC5883L_Addr,HMC5883L_Register_ZMSB);
    BUF[3] = Single_Read(HMC5883L_Addr,HMC5883L_Register_ZLSB);
	BUF[4] = Single_Read(HMC5883L_Addr,HMC5883L_Register_YMSB);
    BUF[5] = Single_Read(HMC5883L_Addr,HMC5883L_Register_YLSB);

    x=(BUF[0] << 8) | BUF[1]; //Combine MSB and LSB of X Data output register
    z=(BUF[2] << 8) | BUF[3]; //Combine MSB and LSB of Z Data output register
    y=(BUF[4] << 8) | BUF[5]; //Combine MSB and LSB of Z Data output register

    if (x > 0x7fff)
        x -= 0xffff;
    if (z > 0x7fff)
        z -= 0xffff;
    if (y > 0x7fff)
        y -= 0xffff;
    *Angle= atan2(y,x) * (180 / 3.14159265) + 180; // angle in degrees
    return 1;
}

/***************************BMP085**********************************/

void Init_BMP085(short *AC_123, unsigned short *AC_456, short *B1_MD)
{
	AC_123[0] = Single_Read(BMP085_Addr,BMP085_AC1);//READ MSB
	AC_123[0] = (AC_123[0]<<8)|Single_Read(BMP085_Addr,BMP085_AC1+ 1);//READ LSB AND COMBINE (MSB | LSB) 

    AC_123[1] = Single_Read(BMP085_Addr,BMP085_AC2);//READ MSB
	AC_123[1] = (AC_123[1]<<8)| Single_Read(BMP085_Addr,BMP085_AC2 + 1);//READ LSB AND COMBINE (MSB | LSB)

	AC_123[2] = Single_Read(BMP085_Addr,BMP085_AC3);//READ MSB
	AC_123[2] = (AC_123[2]<<8)| Single_Read(BMP085_Addr,BMP085_AC3 + 1);//READ LSB AND COMBINE (MSB | LSB)

	AC_456[0] = Single_Read(BMP085_Addr,BMP085_AC4);//READ MSB
	AC_456[0] = (AC_456[0]<<8)| Single_Read(BMP085_Addr,BMP085_AC4 + 1);//READ LSB AND COMBINE (MSB | LSB)

	AC_456[1] = Single_Read(BMP085_Addr,BMP085_AC5);//READ MSB
	AC_456[1] = (AC_456[1]<<8)| Single_Read(BMP085_Addr,BMP085_AC5 + 1);//READ LSB AND COMBINE (MSB | LSB)

	AC_456[2] = Single_Read(BMP085_Addr,BMP085_AC6);//READ MSB
	AC_456[2] = (AC_456[2]<<8)| Single_Read(BMP085_Addr,BMP085_AC6 + 1);//READ LSB AND COMBINE (MSB | LSB)

	B1_MD[0] = Single_Read(BMP085_Addr,BMP085_B1);//READ MSB
	B1_MD[0] = (B1_MD[0]<<8)| Single_Read(BMP085_Addr,BMP085_B1 + 1);//READ LSB AND COMBINE (MSB | LSB)

	B1_MD[1] = Single_Read(BMP085_Addr,BMP085_B2);//READ MSB
	B1_MD[1] = (B1_MD[1]<<8)| Single_Read(BMP085_Addr,BMP085_B2 + 1);//READ LSB AND COMBINE (MSB | LSB)

	B1_MD[2] = Single_Read(BMP085_Addr,BMP085_MB);//READ MSB
	B1_MD[2] = (B1_MD[2]<<8)| Single_Read(BMP085_Addr,BMP085_MB +1);//READ LSB AND COMBINE (MSB | LSB)

	B1_MD[3] = Single_Read(BMP085_Addr,BMP085_MC);//READ MSB
	B1_MD[3] = (B1_MD[3]<<8)| Single_Read(BMP085_Addr,BMP085_MC + 1);//READ LSB AND COMBINE (MSB | LSB)

	B1_MD[4] = Single_Read(BMP085_Addr,BMP085_MD);//READ MSB
	B1_MD[4] = (B1_MD[4]<<8)| Single_Read(BMP085_Addr,BMP085_MD + 1);//READ LSB AND COMBINE (MSB | LSB)

}

int BMP085_READ_TEMPERATURE(long *temperature_temp)
{
    if(!Single_Write(BMP085_Addr, BMP085_DATA_ADDR, BMP085_TEMPERATURE))      return 0;
    ets_delay_us(5 * 1000);//延时5ms(>4.5ms)等待BMP085准备数据
    *temperature_temp = Single_Read(BMP085_Addr, BMP085_DATA_MSB);
    *temperature_temp = ((*temperature_temp) << 8) | Single_Read(BMP085_Addr, BMP085_DATA_LSB);
    return 1;
}

int BMP085_READ_PRESSURE(long *pressure_temp)
{
    if(!Single_Write(BMP085_Addr, BMP085_DATA_ADDR, BMP085_PRESSURE))      return 0;
    ets_delay_us(5 * 1000);//延时5ms(>4.5ms)等待BMP085准备数据
    *pressure_temp = Single_Read(BMP085_Addr, BMP085_DATA_MSB);
    *pressure_temp = ((*pressure_temp) << 8) | Single_Read(BMP085_Addr, BMP085_DATA_LSB);
    *pressure_temp &= 0x0000FFFF;
    return 1;
}

int BMP085_DATA(long *temperature_temp, long *pressure_temp, short *AC_123, unsigned short *AC_456, short *B1_MD)
{
    long ut,up;
	long x1, x2, b5, b6, x3, b3, p;
	unsigned long b4, b7;

	if(!BMP085_READ_TEMPERATURE(&ut))       return 0;	   // 读取温度
	if(!BMP085_READ_PRESSURE(&up))      return 0;  // 读取压强

    //计算temperature
	x1 = ((long)ut - AC_456[2]) * AC_456[1] >> 15;
	x2 = ((long) B1_MD[3] << 11) / (x1 + B1_MD[4]);
	b5 = x1 + x2;
	*temperature_temp = (b5 + 8) >> 4;//此时的temperature输出的值是以0.1℃为单位

    //计算pressure
	b6 = b5 - 4000;
	x1 = (B1_MD[1] * (b6 * b6 >> 12)) >> 11;
	x2 = AC_123[1] * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((long)AC_123[0] * 4 + x3) + 2)/4;
	x1 = AC_123[2] * b6 >> 13;
	x2 = (B1_MD[0] * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (AC_456[0] * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);
	if( b7 < 0x80000000){
        p = (b7 * 2) / b4;
    }    
    else{
        p = (b7 / b4) * 2;
    }
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	*pressure_temp = p + ((x1 + x2 + 3791) >> 4);//此时的pressure输出的值是以Pa为单位

    return 1;
}


/****************************ADXL345**************************************/
int Init_ADXL345(void)
{   
    //读取ADXL345的器件ID 判断是否为ADXL345的默认值：0xE5
    if(Single_Read(ADXL345_Addr, ADXL345_DEVID) != 0xE5)    return 0;

    //以下配置ADXL345的寄存器，见PDF22页
    //全分辨率模式 测量范围,正负16g，13位模式，详见中文手册25-26页
    if(!Single_Write(ADXL345_Addr,ADXL345_DATA_FORMAT, 0x2B))    return 0;
    //设置功率模式：正常功率，数据速率设定为100hz  参考pdf24 13页
    if(!Single_Write(ADXL345_Addr,ADXL345_BW_RATE, 0x0A))    return 0;   
    //使能 DATA_READY 中断
    if(!Single_Write(ADXL345_Addr,ADXL345_INT_ENABLE, 0x80))    return 0; 
    // //X 偏移量 根据测试传感器的状态写入pdf29页
    // if(!Single_Write(ADXL345_Addr,ADXL345_OFSX, 0x00))    return 0; 
    // //Y 偏移量 根据测试传感器的状态写入pdf29页
    // if(!Single_Write(ADXL345_Addr,ADXL345_OFSY, 0x00))    return 0; 
    // //Z 偏移量 根据测试传感器的状态写入pdf29页
    // if(!Single_Write(ADXL345_Addr,ADXL345_OFSZ, 0x00))    return 0;
    // //选择电源模式为测量模式   参考pdf24页
    if(!Single_Write(ADXL345_Addr,ADXL345_POWER_CTL, 0x08))    return 0; 
    return 1;
}

void ADXL345_Read(int *X, int *Y, int *Z)
{
    uint8_t BUF_ADXL345[8];
    BUF_ADXL345[0] = Single_Read(ADXL345_Addr, ADXL345_DATAX0); // OUT_X_L_A
    BUF_ADXL345[1] = Single_Read(ADXL345_Addr, ADXL345_DATAX1); // OUT_X_H_A

    BUF_ADXL345[2] = Single_Read(ADXL345_Addr, ADXL345_DATAY0); // OUT_Y_L_A
    BUF_ADXL345[3] = Single_Read(ADXL345_Addr, ADXL345_DATAY1); // OUT_Y_H_A

    BUF_ADXL345[4] = Single_Read(ADXL345_Addr, ADXL345_DATAZ0); // OUT_Z_L_A
    BUF_ADXL345[5] = Single_Read(ADXL345_Addr, ADXL345_DATAZ1); // OUT_Z_H_A

    BUF_ADXL345[6] = Single_Read(ADXL345_Addr, ADXL345_DEVID);
    *X = (BUF_ADXL345[1] << 8) + BUF_ADXL345[0]; //合成数据
    *Y = (BUF_ADXL345[3] << 8) + BUF_ADXL345[2]; //合成数据
    *Z = (BUF_ADXL345[5] << 8) + BUF_ADXL345[4]; //合成数据
}

void ADXL345_Read_Average(int *X, int *Y, int *Z, int times)
{
    int temp_x_1, temp_y_1, temp_z_1;
    int temp_x_2 = 0, temp_y_2 = 0, temp_z_2 = 0;
    *X = 0;*Y = 0;*Z = 0;
    if(times > 0){
        for(int i = 0; i < times; i++)
        {
            for(int j = 0; j < 10; j++)
            {
                ADXL345_Read(&temp_x_1, &temp_y_1, &temp_z_1);
                temp_x_2 += temp_x_1;
                temp_y_2 += temp_y_1;
                temp_z_2 += temp_z_1;
            }
            temp_x_2 /= 10;
            temp_y_2 /= 10;
            temp_z_2 /= 10;
            *X += temp_x_2;
            *Y += temp_y_2;
            *Z += temp_z_2;

        }
        *X /= times;
        *Y /= times;
        *Z /= times;
    }
}

//******************ADXL345计算倾斜角度************
float ADXL345_Angle(float A_X, float A_Y, float A_Z, int i)
{
    float temp;
    float res = 0;
    switch (i)
    {
    case 0: //与自然Z轴的角度
        // temp = A_Z / sqrt((A_X * A_X + A_Y * A_Y));
        temp = sqrt((A_X * A_X + A_Y * A_Y)) / A_Z;
        res = atan(temp); //反正切
        break;
    case 1: //与自然X轴的角度
        temp = A_X / sqrt((A_Y * A_Y + A_Z * A_Z));
        // temp = sqrt((A_Y * A_Y + A_Z * A_Z)) / A_X;

        res = atan(temp);
        break;
    case 2: //与自然Y轴的角度
        temp = A_Y / sqrt((A_X * A_X + A_Z * A_Z));
        // temp = sqrt((A_X * A_X + A_Z * A_Z)) /  A_Y;
        res = atan(temp);
        break;
    }
    return res * 180 / 3.14;
}

int ADXL345_AUTO_Adjust(void)
{
    int temp_x, temp_y, temp_z;
	int offx=0, offy=0, offz=0;

    //读取ADXL345的器件ID 判断是否为ADXL345的默认值：0xE5
    if(Single_Read(ADXL345_Addr, ADXL345_DEVID) != 0xE5)    return 0;
    //先进入休眠模式.
    if(!Single_Write(ADXL345_Addr,ADXL345_POWER_CTL, 0x00))    return 0;	
    ets_delay_us(100*1000);
    //低电平中断输出,13位全分辨率,输出数据右对齐,16g量程 
	if(!Single_Write(ADXL345_Addr,ADXL345_DATA_FORMAT, 0X2B))    return 0;	
    //数据输出速度为100Hz
	if(!Single_Write(ADXL345_Addr,ADXL345_BW_RATE, 0x0A))    return 0;  	
    //链接使能,测量模式
	if(!Single_Write(ADXL345_Addr,ADXL345_POWER_CTL, 0x28))    return 0;
    //不使用中断		 
    if(!Single_Write(ADXL345_Addr,ADXL345_INT_ENABLE, 0x00))    return 0;
	//X 偏移量 根据测试传感器的状态写入pdf29页
    if(!Single_Write(ADXL345_Addr,ADXL345_OFSX, 0x00))    return 0; 
    //Y 偏移量 根据测试传感器的状态写入pdf29页
    if(!Single_Write(ADXL345_Addr,ADXL345_OFSY, 0x00))    return 0; 
    //Z 偏移量 根据测试传感器的状态写入pdf29页
    if(!Single_Write(ADXL345_Addr,ADXL345_OFSZ, 0x00))    return 0;
    ets_delay_us(12*1000);

    ADXL345_Read_Average(&temp_x, &temp_y, &temp_z, 100);
    offx = -temp_x / 4;
    offy = -temp_y / 4;
    offz = -(temp_z - 256) / 4;
    //X 偏移量 根据测试传感器的状态写入pdf29页
    if(!Single_Write(ADXL345_Addr,ADXL345_OFSX, offx))    return 0; 
    //Y 偏移量 根据测试传感器的状态写入pdf29页
    if(!Single_Write(ADXL345_Addr,ADXL345_OFSY, offy))    return 0; 
    //Z 偏移量 根据测试传感器的状态写入pdf29页
    if(!Single_Write(ADXL345_Addr,ADXL345_OFSZ, offz))    return 0;

    return 1;
}


