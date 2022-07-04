#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"
#include "math.h"

#include "i2c_hardware.h"
#include "register.h"
#include "init_Read.h"

#define OSS 3 // BMP085 使用
/***************************L3G4200D**********************************/
/**
 * @brief           初始化角速度传感器L3G4200D
 * @param   无
 * @return
 *                  1：初始化成功
 *                  0：初始化失败
 */
int Init_L3G4200D(float *drift_buf)
{
    float buf_temp[3] = {0};
    if (Single_Read_hardware(L3G4200_Addr, L3G4200_DEVID) != 0xD3)
        return 0;
    //数据输出速率400Hz
    Single_Write_hardware(L3G4200_Addr, L3G4200_CTRL_REG1, 0x8F);
    Single_Write_hardware(L3G4200_Addr, L3G4200_CTRL_REG2, 0x00);
    // I2_DRDY、I2_WTM、I2_ORun----enable
    Single_Write_hardware(L3G4200_Addr, L3G4200_CTRL_REG3, 0x0E);
    // //250dps
    // Single_Write_hardware(L3G4200_Addr,L3G4200_CTRL_REG4,0x00);
    // 500dps
    Single_Write_hardware(L3G4200_Addr, L3G4200_CTRL_REG4, 0x10);
    // //2000dps
    // Single_Write_hardware(L3G4200_Addr, L3G4200_CTRL_REG4, 0x30);
    // FIFO_EN、HP_EN------enable
    Single_Write_hardware(L3G4200_Addr, L3G4200_CTRL_REG5, 0x10);
    Single_Write_hardware(L3G4200_Addr, L3G4200_REFERENCE, 0x00);
    // FIFO-流模式，样本位：31
    Single_Write_hardware(L3G4200_Addr, L3G4200_FIFO_CTRL, 0x5F);
    L3G4200D_Read_Average(drift_buf, 16, buf_temp);
    return 1;
}

/**
 * @brief                   单次读取L3G4200D的数据
 * @param   BUF_L3G4200D    用于存储本次读取到的数据的数组
 */
void L3G4200D_Read(float *BUF_L3G4200D)
{
    uint8_t BUF[6]= {0,0,0,0,0,0};
    int T_X, T_Y, T_Z;
    //L3G4200D在连读时，需要把寄存器地址最高位置一，即  (register | 0x80)
    Multiple_Read_Hardware(L3G4200_Addr, (L3G4200_OUT_X_L | 0x80), BUF, 6);
    
        T_X = (BUF[1] << 8) | BUF[0];
        T_Y = (BUF[3] << 8) | BUF[2];
        T_Z = (BUF[5] << 8) | BUF[4];

        if (T_X > 0x7FFF)
            T_X -= 0xFFFF;
        if (T_Y > 0x7FFF)
            T_Y -= 0xFFFF;
        if (T_Z > 0x7FFF)
            T_Z -= 0xFFFF;

        // FS = 250dps   * 0.00875
        // FS = 500dps   * 0.0175
        // FS = 2000dps  * 0.07
        BUF_L3G4200D[0] = (float)T_X * 0.0175;
        BUF_L3G4200D[1] = (float)T_Y * 0.0175;
        BUF_L3G4200D[2] = (float)T_Z * 0.0175;
}

/**
 * @brief                   多次读取L3G4200D的数据
 * @param   BUF_L3G4200D    用于存储最终数据的数组
 * @param   times           读取次数：times
 */
void L3G4200D_Read_Average(float *BUF_L3G4200D, int times, float *drift_buf)
{
    float BUF[3] = {0, 0, 0};
    float BUF_TEMP[3] = {0, 0, 0};
    for (int i = 0; i < times; i++)
    {
        L3G4200D_Read(BUF_TEMP);
        BUF[0] += BUF_TEMP[0];
        BUF[1] += BUF_TEMP[1];
        BUF[2] += BUF_TEMP[2];
    }
    BUF_L3G4200D[0] = (BUF[0] / times) - drift_buf[0];
    BUF_L3G4200D[1] = (BUF[1] / times) - drift_buf[1];
    BUF_L3G4200D[2] = (BUF[2] / times) - drift_buf[2];

}

/****************************ADXL345**************************************/
/**
 * @brief       初始化加速度传感器ADXL345
 * @param   无
 * @return
 *              1：初始化成功
 *              0：初始化失败
 */
int Init_ADXL345(void)
{
    //读取ADXL345的器件ID 判断是否为ADXL345的默认值：0xE5
    if (Single_Read_hardware(ADXL345_Addr, ADXL345_DEVID) != 0xE5)
        return 0;

    //以下配置ADXL345的寄存器，见PDF22页
    //关闭中断
    Single_Write_hardware(ADXL345_Addr, ADXL345_INT_ENABLE, 0x00);
    //全分辨率模式 测量范围,正负4g，13位模式，详见中文手册25-26页
    Single_Write_hardware(ADXL345_Addr, ADXL345_DATA_FORMAT, 0x09);
    //设置功率模式：正常功率，数据速率设定为800hz  参考pdf24 13页
    Single_Write_hardware(ADXL345_Addr, ADXL345_BW_RATE, 0x0D);
    //选择电源模式为测量模式   参考pdf24页
    Single_Write_hardware(ADXL345_Addr, ADXL345_POWER_CTL, 0x08);
    //设置FIFO模式:FIFO模式，样本位：31
    Single_Write_hardware(ADXL345_Addr, ADXL345_FIFO_CTL, 0x9F);
    // X 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSX, 0x00);
    // Y 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSY, 0x00);
    // Z 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSZ, 0x00);
    return 1;
}

/**
 * @brief       加速度传感器ADXL345自校准
 * @param   无
 * @return
 *              1：校准成功
 *              0：校准失败
 */
int ADXL345_Auto_Adjust(void)
{
    short temp_buf[3] = {0};
    short temp_x_2 = 0, temp_y_2 = 0, temp_z_2 = 0;
    short offx = 0, offy = 0, offz = 0;

    //读取ADXL345的器件ID 判断是否为ADXL345的默认值：0xE5
    if (Single_Read_hardware(ADXL345_Addr, ADXL345_DEVID) != 0xE5)
        return 0;

    //以下配置ADXL345的寄存器，见PDF22页
    //先进入休眠模式.
    Single_Write_hardware(ADXL345_Addr, ADXL345_POWER_CTL, 0x00);
    ets_delay_us(100 * 1000);

    //高电平中断输出,13位全分辨率,输出数据右对齐,4g量程
    Single_Write_hardware(ADXL345_Addr, ADXL345_DATA_FORMAT, 0X09);
    //链接使能,测量模式
    Single_Write_hardware(ADXL345_Addr, ADXL345_POWER_CTL, 0x28);
    //关闭中断
    Single_Write_hardware(ADXL345_Addr, ADXL345_INT_ENABLE, 0x00);
    // X 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSX, 0x00);
    // Y 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSY, 0x00);
    // Z 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSZ, 0x00);
    ets_delay_us(12 * 1000);

    for (int i = 0; i < 10; i++)
    {
        ADXL345_Read(temp_buf);
        temp_x_2 += temp_buf[0];
        temp_y_2 += temp_buf[1];
        temp_z_2 += temp_buf[2];
    }
    temp_buf[0] = temp_x_2 / 10;
    temp_buf[1] = temp_y_2 / 10;
    temp_buf[2] = temp_z_2 / 10;

    offx = -temp_buf[0] / 4;
    offy = -temp_buf[1] / 4;
    offz = -(temp_buf[2] - 256) / 4;

    printf("%d, %d, %d-----------------------\n", offx, offy, offz);
    // X 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSX, offx);
    // Y 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSY, offy);
    // Z 偏移量 根据测试传感器的状态写入pdf29页
    Single_Write_hardware(ADXL345_Addr, ADXL345_OFSZ, offz);

    return 1;
}

/**
 * @brief           读取加速度传感器ADXL345一次
 * @param   Buf     三轴原始数据
 */
void ADXL345_Read(short *Buf)
{
    uint8_t BUF_ADXL345[6];
    Multiple_Read_Hardware(ADXL345_Addr, ADXL345_DATAX0, BUF_ADXL345, 6);
    Buf[0] = (short)(((uint16_t)BUF_ADXL345[1] << 8) + BUF_ADXL345[0]); //合成数据
    Buf[1] = (short)(((uint16_t)BUF_ADXL345[3] << 8) + BUF_ADXL345[2]); //合成数据
    Buf[2] = (short)(((uint16_t)BUF_ADXL345[5] << 8) + BUF_ADXL345[4]); //合成数据

}

/**
 * @brief           FIFO读取ADXL345加速度传感器
 * 
 * @param Buf       三轴原始数据
 * @param times     读取多少次
 */
void ADXL345_Fifo_Read(float *Buf, int times)
{
    uint8_t BUF_ADXL345[6];
    short Buf_temp[6] = {0};
    short Buf_temp1[3] = {0};
    for(int i = 0; i < times; i++)
    {
        Multiple_Read_Hardware(ADXL345_Addr, ADXL345_DATAX0, BUF_ADXL345, 6);
        for(int i = 0; i < 6; i++)
        {
            Buf_temp[i] += BUF_ADXL345[i];
        }
    }
    //此处times应取多少，Buf_temp1中的值右移多少位，参考网页：https://www.analog.com/en/design-center/interactive-design-tools/accelerometer-fifo-calculator.html
    Buf_temp1[0] = (short)(((uint16_t)Buf_temp[1] << 8) + Buf_temp[0]) >> 2; //合成数据
    Buf_temp1[1] = (short)(((uint16_t)Buf_temp[3] << 8) + Buf_temp[2]) >> 2; //合成数据
    Buf_temp1[2] = (short)(((uint16_t)Buf_temp[5] << 8) + Buf_temp[4]) >> 2; //合成数据
    Buf[0] = ((float)Buf_temp1[0] * 9.8) / 1000.0;
    Buf[1] = ((float)Buf_temp1[1] * 9.8) / 1000.0;
    Buf[2] = ((float)Buf_temp1[2] * 9.8) / 1000.0;

}
/**
 * @brief           多次读取ADXL345传感器，并对三轴原始数据取平均
 * 
 * @param Buf       平均后的三轴原始数据
 * @param times     读取多少次取平均
 */
void ADXL345_Read_Average(float *Buf, int times)
{
    short temp_buf[3] = {0};
    short temp_x = 0, temp_y = 0, temp_z = 0;
    for (int i = 0; i < times; i++)
    {
        ADXL345_Read(temp_buf);
        temp_x += temp_buf[0];
        temp_y += temp_buf[1];
        temp_z += temp_buf[2];
    }
    Buf[0] = (((float)temp_x / times) * 3.9 * 9.8) / 1000.0;
    Buf[1] = (((float)temp_y / times) * 3.9 * 9.8) / 1000.0;
    Buf[2] = (((float)temp_z / times) * 3.9 * 9.8) / 1000.0;
    
}

/**
 * @brief       由ADXL345加速度数据计算偏角
 * 
 * @param Buf   三轴原始数据
 * @param i 
 *              0：与自然Z轴的角度
 *              1：与自然X轴的角度
 *              2：与自然Y轴的角度
 *  * @return float 偏角
 */
float ADXL345_Angle(float *Buf, int i)
{
    float temp;
    float res = 0;
    switch (i)
    {
    case 0: //与自然Z轴的角度
        temp = sqrt((Buf[0] * Buf[0]+ Buf[1] * Buf[1])) / Buf[2];
        res = atan(temp); //反正切
        break;
    case 1: //与自然X轴的角度
        temp = Buf[0] / sqrt((Buf[1] * Buf[1] + Buf[2] * Buf[2]));
        res = atan(temp);
        break;
    case 2: //与自然Y轴的角度
        temp = Buf[1] / sqrt((Buf[0] * Buf[0] + Buf[2] * Buf[2]));
        res = atan(temp);
        break;
    }
    return res * 180 / 3.14159265;
}

/***************************HMC5883L**********************************/
/**
 * @brief   初始化磁场传感器HMC5883L
 * @param   无
 * @return  无
 */
void Init_HMC5883L(void)
{
    //数据输出速率 75Hz  01011000
    
    Single_Write_hardware(HMC5883L_Addr, HMC5883L_Configuration_Register_A, 0x78);
    Single_Write_hardware(HMC5883L_Addr, HMC5883L_Configuration_Register_B, 0x20);
    Single_Write_hardware(HMC5883L_Addr, HMC5883L_Mode_Register, 0x00);
}


/**
 * @brief           读取HMC5883L原始数据
 * 
 * @param mag_XYZ   HMC5883L的三轴原始数据
 */
void HMC5883L_Raw_Read(float *mag_XYZ)
{
    uint8_t BUF[6] = {0};
    if ((Single_Read_hardware(HMC5883L_Addr, HMC5883L_Status_Register) & 0x01) == 0x01)
    {
        Multiple_Read_Hardware(HMC5883L_Addr, HMC5883L_Register_XMSB, BUF, 6);
        mag_XYZ[0] = (BUF[0] << 8) | BUF[1]; // Combine MSB and LSB of X Data output register
        mag_XYZ[2] = (BUF[2] << 8) | BUF[3]; // Combine MSB and LSB of Z Data output register
        mag_XYZ[1] = (BUF[4] << 8) | BUF[5]; // Combine MSB and LSB of Y Data output register
        if (mag_XYZ[0] > 0x7FFF)
            mag_XYZ[0] -= 0xFFFF;
        if (mag_XYZ[2] > 0x7FFF)
            mag_XYZ[2] -= 0xFFFF;
        if (mag_XYZ[1] > 0x7FFF)
            mag_XYZ[1] -= 0xFFFF;
        // mag_XYZ[0] /= 1090.0f;
        // mag_XYZ[1] /= 1090.0f;
        // mag_XYZ[2] /= 1090.0f;
        // printf("%f, %f, %f\n", mag_XYZ[0], mag_XYZ[1], mag_XYZ[2]);
    }
}

/**
 * @brief           HMC5883L自测校准
 * 
 * @param Offset    偏置
 * @param K_XYZ     比例系数
 */
void HMC5883L_SELFTEST(float *Offset, float *K_XYZ)
{
    float xyz[3];
    float GaXmax = 0, GaXmin = 0, GaYmax = 0, GaYmin = 0, GaZmax = 0, GaZmin = 0;
    printf("Start HMC5883L Selftest!!!!!!\n");
    vTaskDelay(2000 / portTICK_RATE_MS);
    for(int i = 0; i < 500; i++)
    {
        HMC5883L_Raw_Read(xyz);
        GaXmax = GaXmax < xyz[0]? xyz[0]:GaXmax;
		GaXmin = GaXmin > xyz[0]? xyz[0]:GaXmin;
		GaYmax = GaYmax < xyz[1]? xyz[1]:GaYmax;
		GaYmin = GaYmin > xyz[1]? xyz[1]:GaYmin;
        GaZmax = GaZmax < xyz[2]? xyz[2]:GaZmax;
		GaZmin = GaZmin > xyz[2]? xyz[2]:GaZmin;
        ets_delay_us(10000);
    }
    printf("%f, %f, %f, %f, %f, %f\n",GaXmax, GaXmin, GaYmax, GaYmin, GaZmax, GaZmin);
    printf("\nHMC5883L Selftest End!!!!!!\n");
    Offset[0] = (GaXmax + GaXmin) / 2;
    Offset[1] = (GaYmax + GaYmin) / 2;
    Offset[2] = (GaZmax + GaZmin) / 2;
    K_XYZ[0] = 2 / (GaXmax - GaXmin);
    K_XYZ[1] = 2 / (GaYmax - GaYmin);
    K_XYZ[2] = 2 / (GaZmax - GaZmin);
    printf("%f, %f, %f, %f, %f, %f\n",Offset[0], Offset[1], Offset[2], K_XYZ[0], K_XYZ[1], K_XYZ[2]);
}

/**
 * @brief           HMC5883L数据读取
 * @param Angle     存储三轴偏角
 * @param mag_XYZ   存储三轴原始数据
 * @param Offset    用于校准的数据
 * @param K_XYZ     用于校准的数据
 */
void HMC5883L_READ(float *Angle, float *mag_XYZ, float *Offset, float *K_XYZ)
{
    float rawGaXYZ[3];
    HMC5883L_Raw_Read(rawGaXYZ);
    mag_XYZ[0] = (rawGaXYZ[0] - Offset[0]) * K_XYZ[0];
    mag_XYZ[1] = (rawGaXYZ[1] - Offset[1]) * K_XYZ[1];
    mag_XYZ[2] = (rawGaXYZ[2] - Offset[2]) * K_XYZ[2];
    // mag_XYZ[0] = rawGaXYZ[0] - Offset[0];
    // mag_XYZ[1] = rawGaXYZ[1] - Offset[1];
    // mag_XYZ[2] = rawGaXYZ[2] - Offset[2];
    // mag_XYZ[2] = rawGaXYZ[2];
    // Angle[0]= atan2(mag_XYZ[1],mag_XYZ[0]) * (180 / 3.14159265) + 180; // angle in degrees
    // Angle[1]= atan2(mag_XYZ[2],mag_XYZ[0]) * (180 / 3.14159265) + 180; // angle in degrees
    // Angle[2]= atan2(mag_XYZ[2],mag_XYZ[1]) * (180 / 3.14159265) + 180; // angle in degrees
}

/***************************BMP085**********************************/

/**
 * @brief           初始化用于辅助计算温度、气压值的参数值数组
 * @param   AC_123  存储用于校准的基础数据
 * @param   AC_456  存储用于校准的基础数据
 * @param   B1_MD   存储用于校准的基础数据
 */
void Init_BMP085(short *AC_123, unsigned short *AC_456, short *B1_MD)
{
    AC_123[0] = Single_Read_hardware(BMP085_Addr, BMP085_AC1);                        // READ MSB
    AC_123[0] = (AC_123[0] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_AC1 + 1); // READ LSB AND COMBINE (MSB | LSB)
    AC_123[1] = Single_Read_hardware(BMP085_Addr, BMP085_AC2);                        // READ MSB
    AC_123[1] = (AC_123[1] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_AC2 + 1); // READ LSB AND COMBINE (MSB | LSB)
    AC_123[2] = Single_Read_hardware(BMP085_Addr, BMP085_AC3);                        // READ MSB
    AC_123[2] = (AC_123[2] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_AC3 + 1); // READ LSB AND COMBINE (MSB | LSB)
    AC_456[0] = Single_Read_hardware(BMP085_Addr, BMP085_AC4);                        // READ MSB
    AC_456[0] = (AC_456[0] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_AC4 + 1); // READ LSB AND COMBINE (MSB | LSB)
    AC_456[1] = Single_Read_hardware(BMP085_Addr, BMP085_AC5);                        // READ MSB
    AC_456[1] = (AC_456[1] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_AC5 + 1); // READ LSB AND COMBINE (MSB | LSB)
    AC_456[2] = Single_Read_hardware(BMP085_Addr, BMP085_AC6);                        // READ MSB
    AC_456[2] = (AC_456[2] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_AC6 + 1); // READ LSB AND COMBINE (MSB | LSB)
    B1_MD[0] = Single_Read_hardware(BMP085_Addr, BMP085_B1);                       // READ MSB
    B1_MD[0] = (B1_MD[0] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_B1 + 1); // READ LSB AND COMBINE (MSB | LSB)
    B1_MD[1] = Single_Read_hardware(BMP085_Addr, BMP085_B2);                       // READ MSB
    B1_MD[1] = (B1_MD[1] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_B2 + 1); // READ LSB AND COMBINE (MSB | LSB)
    B1_MD[2] = Single_Read_hardware(BMP085_Addr, BMP085_MB);                       // READ MSB
    B1_MD[2] = (B1_MD[2] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_MB + 1); // READ LSB AND COMBINE (MSB | LSB)
    B1_MD[3] = Single_Read_hardware(BMP085_Addr, BMP085_MC);                       // READ MSB
    B1_MD[3] = (B1_MD[3] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_MC + 1); // READ LSB AND COMBINE (MSB | LSB)
    B1_MD[4] = Single_Read_hardware(BMP085_Addr, BMP085_MD);                       // READ MSB
    B1_MD[4] = (B1_MD[4] << 8) | Single_Read_hardware(BMP085_Addr, BMP085_MD + 1); // READ LSB AND COMBINE (MSB | LSB)
}

/**
 * @brief                       磁场传感器HMC5883L温度数据读取
 * @param   temperature_temp    存储原始温度数据
 */
void BMP085_Read_Temperature(long *temperature_temp)
{
    Single_Write_hardware(BMP085_Addr, BMP085_DATA_ADDR, BMP085_TEMPERATURE);
    ets_delay_us(5 * 1000); //延时5ms(>4.5ms)等待BMP085准备数据
    *temperature_temp = Single_Read_hardware(BMP085_Addr, BMP085_DATA_MSB);
    *temperature_temp = ((*temperature_temp) << 8) | Single_Read_hardware(BMP085_Addr, BMP085_DATA_LSB);

}

/**
 * @brief                   磁场传感器HMC5883L气压数据读取
 * @param   pressure_temp   存储原始气压数据
 */
void BMP085_Read_Pressure(long *pressure_temp)
{
    long temp[3] = {0};
    Single_Write_hardware(BMP085_Addr, BMP085_DATA_ADDR, BMP085_PRESSURE + (OSS << 6));
    ets_delay_us(26 * 1000); //延时26ms(>25.5ms)等待BMP085准备数据
    temp[0] = Single_Read_hardware(BMP085_Addr, BMP085_DATA_MSB);
    temp[1] = Single_Read_hardware(BMP085_Addr, BMP085_DATA_LSB);
    temp[2] = Single_Read_hardware(BMP085_Addr, BMP085_DATA_XLSB);
    *pressure_temp = ((temp[0] << 16) | (temp[1] << 8) | temp[2]) >> (8 - OSS);
    *pressure_temp &= 0x0007FFFF;//将19位以后都置0

}

/**
 * @brief                       磁场传感器HMC5883L温度、气压数据计算
 * @param   temperature_temp    存储温度数据
 * @param   pressure_temp       存储气压数据
 */
void BMP085_Data_Calculate(long *temperature_temp, long *pressure_temp, short *AC_123, unsigned short *AC_456, short *B1_MD)
{
    long ut, up = 0;
    long x1, x2, b5, b6, x3, b3, p;
    unsigned long b4, b7;

    BMP085_Read_Temperature(&ut);// 读取温度
    BMP085_Read_Pressure(&up);// 读取压强

    //计算temperature
    x1 = ((long)ut - AC_456[2]) * AC_456[1] >> 15;
    x2 = ((long)B1_MD[3] << 11) / (x1 + B1_MD[4]);
    b5 = x1 + x2;
    *temperature_temp = (b5 + 8) >> 4; //此时的temperature输出的值是以0.1℃为单位

    //计算pressure
    b6 = b5 - 4000;
    x1 = (B1_MD[1] * (b6 * b6 >> 12)) >> 11;
    x2 = AC_123[1] * b6 >> 11;
    x3 = x1 + x2;
    b3 = ((((long)AC_123[0] * 4 + x3) << OSS) + 2) / 4;
    x1 = AC_123[2] * b6 >> 13;
    x2 = (B1_MD[0] * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (AC_456[0] * (unsigned long)(x3 + 32768)) >> 15;
    b7 = ((unsigned long)up - b3) * (50000 >> OSS);
    if (b7 < 0x80000000)
    {
        p = (b7 * 2) / b4;
    }
    else
    {
        p = (b7 / b4) * 2;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    *pressure_temp = p + ((x1 + x2 + 3791) >> 4); //此时的pressure输出的值是以Pa为单位

}
