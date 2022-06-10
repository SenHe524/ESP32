#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "math.h"
#include "esp_timer.h"
#include "Init_Read.h"
#include "Register.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "IMU.h"
#include "KF.h"
/******************************变量定义*********************************/
typedef struct
{
    // L3G4200D
    float Buf_Gyro[3]; //三轴角速度
    // HMC5883L
    float mag_XYZ[3];        //合成的原始XYZ三轴数据
    float mag_Angle[3];      //计算出的三个平面偏角
    float Offset[3];         //自测偏移
    float K_XYZ[3];
    // //BMP0885
    short AC_123[3];
    unsigned short AC_456[3];
    short B1_MD[5];
    long temperature, pressure;
    // ADXL345
    float ACC_Angle[3]; //三轴偏转角
    float ACC_Gavity[3]; //三轴加速度
    float Kalman_Angle[3];
} AHRS_DATA_t;

AHRS_DATA_t Data = {
    .Kalman_Angle[0] = 0,
    .Kalman_Angle[1] = 0,
    .Kalman_Angle[2] = 0,
};

typedef struct
{
    float Roll;  //横滚角
    float Pitch; //俯仰角
    float Yaw;   //航向角
} FLOAT_ANGLE_t;

float Q_angle[3] = {0};
const char *TAG = "GY-80";

/*************************任务句柄及回调函数声明**************************/
static TaskHandle_t Task_AHRS_t = NULL;
static TaskHandle_t Task_Show_t = NULL;
void Task_AHRS(void *parameter);
void Task_Show(void *parameter);
/*************************数据读取函数声明**************************/
void Init_AHRS(void);
void L3G4200D_Data_Update(void);
void ADXL345_Data_Update(void);
void HMC5883L_Data_Update(void);
void BMP085_Data_Update(void);

/******************************主函数*********************************/
void app_main(void)
{
    esp_task_wdt_init(1000, false);
    esp_task_wdt_add(Task_AHRS_t);
    //初始化BMP085
    Init_BMP085(Data.AC_123, Data.AC_456, Data.B1_MD);
    printf("BMP085 初始化完成!\n");
    xTaskCreate((TaskFunction_t)Task_AHRS, "Task_AHRS", 8192, &Data, 10, &Task_AHRS_t);
    xTaskCreate((TaskFunction_t)Task_Show, "Task_Show", 4096, &Data, 5, &Task_Show_t);
}
/***************************Init AHRS*********************************/
void Init_AHRS(void)
{
    //初始化ADXL345
    while (!Init_ADXL345());
    while (!ADXL345_Auto_Adjust());
    while (!Init_ADXL345());
    printf("ADXL345 初始化完成!\n");
    //初始化L3G4200D
    while (!Init_L3G4200D());
    printf("\nL3G4200D 初始化完成!\n");
    //初始化HMC5883L
    while (!Init_HMC5883L());
    printf("HMC5883L 初始化完成!\n");
    // HMC5883L_SELFTEST(Data.Offset, Data.K_XYZ);
}
/***************************AHRS*********************************/
void Task_AHRS(void *parameter)
{
    // int i = 0;
    Init_AHRS();

    while (1)
    {
        // printf("当前时间为：%lld------------------------1\n", esp_timer_get_time());
        /*****************读取加速度*******************/
        ADXL345_Data_Update();
        // printf("当前时间为：%lld------------------------2\n", esp_timer_get_time());
        /*****************读取角速度*******************/
        L3G4200D_Data_Update();
        // printf("当前时间为：%lld------------------------3\n", esp_timer_get_time());
        /*****************读取磁场角度*******************/
        HMC5883L_Data_Update();

        // Data.ACC_Angle[0] = kalmanCalculate(Data.ACC_Angle[0], Data.Buf_Gyro[0], 9);
        // Data.ACC_Angle[1] = kalmanCalculate(Data.ACC_Angle[1], Data.Buf_Gyro[1], 9);
        // Data.ACC_Angle[2] = kalmanCalculate(Data.ACC_Angle[2], Data.Buf_Gyro[2], 9);
        // printf("ACC_Angle = %f, ACC_Angle = %f, ACC_Angle = %f---------------------2\n", Data.ACC_Angle[0], Data.ACC_Angle[1], Data.ACC_Angle[2]);

        IMU_AHRSupdate(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
                        Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2],
                        Data.mag_XYZ[0], Data.mag_XYZ[1], Data.mag_XYZ[2], Q_angle);
        // AHRSupdate(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
        //                 Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2],
        //                 Data.mag_XYZ[0], Data.mag_XYZ[1], Data.mag_XYZ[2], Q_angle);
        // MadgwickAHRSupdate(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
        //                 Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2],
        //                 Data.mag_XYZ[0], Data.mag_XYZ[1], Data.mag_XYZ[2], Q_angle);
        // MadgwickAHRSupdateIMU(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
        //                     Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2], Q_angle);//有一定作用
        // MahonyAHRSupdate(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
        //                     Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2],
        //                     Data.mag_XYZ[0], Data.mag_XYZ[1], Data.mag_XYZ[2], Q_angle);
        // MahonyAHRSupdateIMU(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
        //                     Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2], Q_angle);
        // printf("当前时间为：%lld------------------------5\n", esp_timer_get_time());
        esp_task_wdt_reset();

    }
}

void Task_Show(void *parameter)
{
    
    while(1)
    {
        printf("\n");
        // BMP085_Data_Update();
        ESP_LOGI(TAG, "Roll: %f\n", Q_angle[0]);
        ESP_LOGI(TAG, "Pitch: %f\n", Q_angle[1]);
        ESP_LOGI(TAG, "Yaw: %f\n", Q_angle[2]);
        ESP_LOGI(TAG, "三轴角速度为：X = %f, Y = %f, Z = %f\n", Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2]);
        ESP_LOGI(TAG, "三轴加速度为：Xg = %lf, Yg = %lf, Zg = %lf\n", Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2]);
        ESP_LOGI(TAG, "偏角为：Angle_X = %lf, Angle_Y = %lf, Angle_Z = %lf\n", Data.ACC_Angle[0], Data.ACC_Angle[1], Data.ACC_Angle[2]);
        ESP_LOGI(TAG, "当前磁场角度为：XoY = %f, XoZ = %f, YoZ = %f\n\n", Data.mag_Angle[0], Data.mag_Angle[1], Data.mag_Angle[2]);
        // ESP_LOGI(TAG, "当前温度为：%.2lf ℃\n", (double)Data.temperature / 10.0);
        // ESP_LOGI(TAG, "当前压强为：%ld Pa\n", Data.pressure);
        // ESP_LOGI(TAG, "当前海拔为：%fm\n", 44330 * (1 - pow(Data.pressure / 101325.0, 1.0 / 5.255)));
        vTaskDelay(1500 / portTICK_RATE_MS);
    }
}

/*************************数据读取函数**************************/
void L3G4200D_Data_Update(void)
{
    L3G4200D_Read_Average(Data.Buf_Gyro, 5);
}
void ADXL345_Data_Update(void)
{
    ADXL345_Read_Average(Data.ACC_Gavity, 5);
    Data.ACC_Angle[0] = ADXL345_Angle(Data.ACC_Gavity, 1);
    Data.ACC_Angle[1] = ADXL345_Angle(Data.ACC_Gavity, 2);
    Data.ACC_Angle[2] = ADXL345_Angle(Data.ACC_Gavity, 0);
}
void HMC5883L_Data_Update(void)
{
    // HMC5883L_READ(Data.BUF_HMC5883L, Data.mag_Angle, Data.mag_XYZ, Data.Offset, Data.K_XYZ);
    HMC5883L_Raw_Read(Data.mag_Angle, Data.mag_XYZ);
}
void BMP085_Data_Update(void)
{
    if (!BMP085_Data_Calculate(&Data.temperature, &Data.pressure, Data.AC_123, Data.AC_456, Data.B1_MD))
    {
        printf("获取压强温度失败！\n");
    }
}
