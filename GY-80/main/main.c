#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"
#include "math.h"
#include "esp_timer.h"
#include "I2C.h"
#include "Init_Read.h"
#include "Register.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "IMU.h"
/******************************变量定义*********************************/
typedef struct
{
    float yaw;
    //L3G4200D
    // int Init_L3G4200D; //初始化标志位
    float BUF_GYRO[3];//三轴角速度
    float BUF_MARK[3];
    //HMC5883L
    uint8_t BUF_HMC5883L[6];//读取到的原始数据
    float mag_XYZ[3];//合成的原始XYZ三轴数据
    float mag_Angle[3];//计算出的三个平面偏角
    float Offset[3];//自测偏移
    float K_XYZ[3];
    // //BMP0885
    short AC_123[3];
    unsigned short AC_456[3];
    short B1_MD[5];
    long temperature, pressure;
    //ADXL345
    int A_XYZ[3];//原始数据
    float ACC_Angle[3];//三轴偏转角
    float ACC_XYZ_G[3];//三轴加速度
    float Kalman_Angle[3];
}  GY_80_DATA_t;

GY_80_DATA_t Data = {
        // .Init_L3G4200D = 0,
        .BUF_MARK[0] = 0,
        .BUF_MARK[1] = 0,
        .BUF_MARK[2] = 0,
        .Kalman_Angle[0] = 0,
        .Kalman_Angle[1] = 0,
        .Kalman_Angle[2] = 0,
    };

typedef struct
{
	float Roll;//横滚角
	float Pitch;//俯仰角
	float Yaw;//航向角
}FLOAT_ANGLE_t;

// #define Kp 0.8f         //Kp大，置信加速度计；Kp小，置信陀螺仪
// #define Ki 0.001f       
// #define halfT 0.005f
FLOAT_ANGLE_t Q_ANGLE;
//******卡尔曼参数************
float Q_angle=0.001;  	//陀螺仪噪声协方差
float Q_gyro=0.003;     //陀螺仪漂移噪声协方差
float R_angle=0.5;		//角度测量噪声协方差
float dt=0.276;	        //dt为kalman滤波器采样时间;
float Q_bias[3]; 		    //陀螺仪漂移
float Angle_err;
float K_0, K_1;
float P[2][2] = { { 1, 0 },{ 0, 1 } };

/*************************任务句柄及回调函数声明**************************/
static TaskHandle_t Task_GY_80_t = NULL;
void Task_GY_80(void *parameter);
/*************************数据读取函数声明**************************/
void Init_GY_80(void);
void L3G4200D_DATA(void);
void ADXL345_DATA(void);
void HMC5883L_DATA(void);
void BMP085_DATA(void);
/*************************卡尔曼滤波函数声明**************************/
void Kalman_Filter(int i);
/******************************主函数*********************************/
void app_main(void)
{
    xTaskCreate((TaskFunction_t)Task_GY_80,"Task_GY_80",8192,&Data,5,&Task_GY_80_t);
}
/***************************Init GY_80*********************************/
void Init_GY_80(void)
{
    //初始化ADXL345
    while(!Init_ADXL345());
    while(!ADXL345_AUTO_Adjust());
    while(!Init_ADXL345());
    //初始化L3G4200D
    while(1){
        Init_L3G4200D(Data.BUF_MARK);
        if((Data.BUF_MARK[0] > 1) || (Data.BUF_MARK[1] > 1) || (Data.BUF_MARK[2] > 1)){
            printf("L3G4200D 初始化失败!\n");
        }
        else{
            for(int i = 0; i < 3; i++)
            {
                Q_bias[i] = Data.BUF_MARK[i];
            }
            break;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
        
    //初始化HMC5883L
    while(!Init_HMC5883L()){
        printf("HMC5883L 初始化失败!\n");
    }
    // HMC5883L_SELFTEST(Data.Offset, Data.K_XYZ);
    //初始化BMP085
    Init_BMP085(Data.AC_123, Data.AC_456, Data.B1_MD);
}
/***************************GY_80*********************************/
void Task_GY_80(void *parameter)
{
    // int i = 0;
    Init_GY_80();
    
    while (1)
    {
        printf("当前时间为：%lld------------------------1\n",esp_timer_get_time());
        /*****************读取加速度*******************/
        ADXL345_DATA();
        // printf("当前时间为：%lld------------------------2\n",esp_timer_get_time());
        /*****************读取角速度*******************/
        L3G4200D_DATA();
        // printf("当前时间为：%lld------------------------3\n",esp_timer_get_time());
        /*****************读取温度和压强*******************/
        BMP085_DATA();
        // printf("当前时间为：%lld------------------------4\n",esp_timer_get_time());
        /*****************读取磁场角度*******************/
        HMC5883L_DATA();
        // printf("当前时间为：%lld------------------------5\n",esp_timer_get_time());
        // Kalman_Filter(0);//对X轴陀螺仪数据进行滤波
        // // printf("当前时间为：%lld------------------------6\n",esp_timer_get_time());
        // Kalman_Filter(1);//对Y轴陀螺仪数据进行滤波
        // // printf("当前时间为：%lld------------------------7\n",esp_timer_get_time());
        // Kalman_Filter(2);//对Z轴陀螺仪数据进行滤波
        printf("当前时间为：%lld------------------------8\n",esp_timer_get_time());
        // printf("Kalman_Angle: X = %f, Y = %f, Z = %f\n",Data.Kalman_Angle[0] ,Data.Kalman_Angle[1] ,Data.Kalman_Angle[2]);
        // printf("三轴角速度为：X = %f, Y = %f, Z = %f\n", Data.BUF_GYRO[0], Data.BUF_GYRO[1],Data.BUF_GYRO[2]);
        IMU_AHRSupdate(Data.BUF_GYRO[0], Data.BUF_GYRO[1], Data.BUF_GYRO[2], 
                        Data.ACC_XYZ_G[0], Data.ACC_XYZ_G[1], Data.ACC_XYZ_G[2],
                        Data.mag_XYZ[0], Data.mag_XYZ[1], Data.mag_XYZ[2]);
        // MadgwickAHRSupdateIMU(Data.BUF_GYRO[0], Data.BUF_GYRO[1], Data.BUF_GYRO[2], 
        //                     Data.ACC_XYZ_G[0], Data.ACC_XYZ_G[1], Data.ACC_XYZ_G[2]);
        // printf("Yaw: %f\n", 0.9 * (Data.yaw - Data.BUF_GYRO[2] * 0.01) + 0.1 * Data.mag_Angle[0]);
        printf("\n\n");
        // printf("----------------------------------%d---------------------------------------------\n", ++i);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

/*************************数据读取函数**************************/
void L3G4200D_DATA(void)
{
    uint8_t temp = 0;
    L3G4200D_READ_AVERAGE(Data.BUF_GYRO, Data.BUF_MARK, 2);
    temp = Single_Read(L3G4200_Addr, L3G4200_TEMP);
    printf("三轴角速度为：X = %f, Y = %f, Z = %f\n", Data.BUF_GYRO[0], Data.BUF_GYRO[1], Data.BUF_GYRO[2]);
    printf("当前温度为：%d℃\n",temp);
}
void ADXL345_DATA(void)
{
    ADXL345_Read_Average(&Data.A_XYZ[0], &Data.A_XYZ[1], &Data.A_XYZ[2], 2);
    Data.ACC_XYZ_G[0] = (((float)Data.A_XYZ[0] * 3.9) / 1000) * 9.8;
    Data.ACC_XYZ_G[1] = (((float)Data.A_XYZ[1] * 3.9) / 1000) * 9.8;
    Data.ACC_XYZ_G[2] = (((float)Data.A_XYZ[2] * 3.9) / 1000) * 9.8;
    Data.ACC_Angle[0] = ADXL345_Angle((float)Data.A_XYZ[0] * 3.9, (float)Data.A_XYZ[1] * 3.9, (float)Data.A_XYZ[2] * 3.9, 1);
    Data.ACC_Angle[1] = ADXL345_Angle((float)Data.A_XYZ[0] * 3.9, (float)Data.A_XYZ[1] * 3.9, (float)Data.A_XYZ[2] * 3.9, 2);
    Data.ACC_Angle[2] = ADXL345_Angle((float)Data.A_XYZ[0] * 3.9, (float)Data.A_XYZ[1] * 3.9, (float)Data.A_XYZ[2] * 3.9, 0);
    printf("三轴加速度为：Xg = %lf, Yg = %lf, Zg = %lf\n", Data.ACC_XYZ_G[0], Data.ACC_XYZ_G[1], Data.ACC_XYZ_G[2]);
    printf("偏角为：Angle_X = %lf, Angle_Y = %lf, Angle_Z = %lf\n", Data.ACC_Angle[0], Data.ACC_Angle[1], Data.ACC_Angle[2]);
}
void HMC5883L_DATA(void)
{
    // HMC5883L_READ(Data.BUF_HMC5883L, Data.mag_Angle, Data.mag_XYZ, Data.Offset, Data.K_XYZ);
    HMC5883L_RAW_READ(Data.BUF_HMC5883L, Data.mag_Angle, Data.mag_XYZ);
    printf("当前磁场角度为：XoY = %f, XoZ = %f, YoZ = %f\n",Data.mag_Angle[0], Data.mag_Angle[1], Data.mag_Angle[2]);

}
void BMP085_DATA(void)
{
    // GY_80_DATA_t *Data = parameter;
    if(BMP085_DATA_CALCULATE(&Data.temperature, &Data.pressure, Data.AC_123, Data.AC_456, Data.B1_MD)){
        printf("当前温度为：%.2lf ℃\n", (double)Data.temperature/10.0);
        printf("当前压强为：%ld Pa\n",Data.pressure);
        // printf("当前海拔为：%fm\n",(101325.0f - Data.pressure)* 9 / 100);
        printf("当前海拔为：%fm\n",44330 * (1 - pow(Data.pressure / 101325.0, 1.0/5.255)));
    }
    else{
    printf("获取压强温度失败！\n");
    }
}

void Kalman_Filter(int i)		
{
    Data.Kalman_Angle[i] += (Data.BUF_GYRO[i] - Q_bias[i]) * dt; //先验估计

    P[0][0] += Q_angle -(P[0][1] + P[1][0]) * dt;
    P[0][1] += -P[1][1] * dt;
    P[1][0] += -P[1][1] * dt;
    P[1][1] += Q_gyro;

    Angle_err = Data.ACC_Angle[i] - Data.Kalman_Angle[i];	//zk-先验估计

    K_0 = P[0][0] / (P[0][0] + R_angle);
    K_1 = P[1][0] / (P[0][0] + R_angle);


    P[0][0] -= K_0 * P[0][0];		 //后验估计误差协方差
    P[0][1] -= K_0 * P[0][1];
    P[1][0] -= K_1 * P[0][0];
    P[1][1] -= K_1 * P[0][1];
        
    Data.Kalman_Angle[i] += K_0 * Angle_err;	 //后验估计
    Q_bias[i] += K_1 * Angle_err;	 //后验估计
    Data.BUF_GYRO[i]   = Data.BUF_GYRO[i] - Q_bias[i];	 //输出值(后验估计)的微分=角速度

}