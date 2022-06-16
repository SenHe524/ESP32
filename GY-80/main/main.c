#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "Init_Read.h"
#include "Register.h"
#include "IMU.h"
#include "KF.h"
#include "i2c_hardware.h"
/******************************变量定义*********************************/
typedef struct
{
    // L3G4200D
    float Buf_Gyro[3]; //三轴角速度
    // ADXL345
    float ACC_Angle[3]; //三轴偏转角
    float ACC_Gavity[3]; //三轴加速度
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
    
} AHRS_DATA_t;

AHRS_DATA_t Data;

typedef struct
{
    float Roll;  //横滚角
    float Pitch; //俯仰角
    float Yaw;   //航向角
} FLOAT_ANGLE_t;

float Q_angle[3] = {0};
float Q_angle_temp[3] = {0};
float kalman_angle[3] = {0};
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
#define I2C_SCL_HARDWARE 18
#define I2C_SDA_HARDWARE 19

void app_main(void)
{
    esp_task_wdt_init(18000, false);
    esp_task_wdt_add(Task_AHRS_t);
    I2C_Init(I2C_SCL_HARDWARE, I2C_SDA_HARDWARE);
    Init_AHRS();
    ADXL345_Data_Update();
    kalmanSetAngle(Data.ACC_Angle[0], 0);
    kalmanSetAngle(Data.ACC_Angle[1], 1);
    kalmanSetAngle(Data.ACC_Angle[2], 2);
    kalman_angle[0] = Data.ACC_Angle[0];
    kalman_angle[1] = Data.ACC_Angle[1];
    kalman_angle[2] = Data.ACC_Angle[2];
    xTaskCreatePinnedToCore((TaskFunction_t)Task_AHRS, "Task_AHRS", 8192, &Data, 10, &Task_AHRS_t, (BaseType_t)1);
    xTaskCreatePinnedToCore((TaskFunction_t)Task_Show, "Task_Show", 4096, &Data, 5, &Task_Show_t, (BaseType_t)0);
}
/***************************Init AHRS*********************************/
void Init_AHRS(void)
{
    //初始化ADXL345
    while (!Init_ADXL345());
    while (!ADXL345_Auto_Adjust());
    printf("ADXL345 初始化完成\n");
    //初始化L3G4200D
    while (!Init_L3G4200D());
    printf("L3G4200D 初始化完成\n");
    //初始化HMC5883L
    Init_HMC5883L();
    HMC5883L_SELFTEST(Data.Offset, Data.K_XYZ);
    printf("HMC5883L 初始化完成\n");
    //初始化BMP085
    Init_BMP085(Data.AC_123, Data.AC_456, Data.B1_MD);
    printf("BMP085 初始化完成\n");
    
}
/***************************AHRS*********************************/
void Task_AHRS(void *parameter)
{
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
        // IMU_AHRSupdate(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
        //                 Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2],
        //                 Data.mag_XYZ[0], Data.mag_XYZ[1], Data.mag_XYZ[2], Q_angle);
        IMU_update(Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2],
                        Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2], Q_angle);
        Q_angle_temp[0] = kalmanCalculate(Q_angle[0], Data.Buf_Gyro[0], 10, 0);
        Q_angle_temp[1] = kalmanCalculate(Q_angle[1], Data.Buf_Gyro[1], 10, 1);
        Q_angle_temp[2] = kalmanCalculate(Q_angle[2], Data.Buf_Gyro[2], 10, 2);
        kalman_angle[0] = 0.952 * (kalman_angle[0] + Data.Buf_Gyro[0] * 1.0f / 100.0f) + 0.048 *  Q_angle_temp[0];
        kalman_angle[1] = 0.952 * (kalman_angle[1] + Data.Buf_Gyro[1] * 1.0f / 100.0f) + 0.048 *  Q_angle_temp[1];
        kalman_angle[2] = 0.952 * (kalman_angle[2] + Data.Buf_Gyro[2] * 1.0f / 100.0f) + 0.048 *  Q_angle_temp[2];
        esp_task_wdt_reset();
        // printf("当前时间为：%lld------------------------5\n", esp_timer_get_time());
        // vTaskDelay(1000 / portTICK_RATE_MS);

    }
}

void Task_Show(void *parameter)
{
    
    while(1)
    {
        printf("\n");
        BMP085_Data_Update();
        ESP_LOGI(TAG, "Roll: %f\t kalman_Roll: %f", Q_angle[0], Q_angle_temp[0]);
        ESP_LOGI(TAG, "Pitch: %f\t kalman_Pitch: %f", Q_angle[1], Q_angle_temp[1]);
        ESP_LOGI(TAG, "Yaw: %f\t kalman_Yaw: %f", Q_angle[2], Q_angle_temp[2]);
        ESP_LOGI(TAG, "比例融合后的Kalman姿态角为：Roll = %lf, Pitch = %lf, Yaw = %lf\n", kalman_angle[0], kalman_angle[1], kalman_angle[2]);

        ESP_LOGI(TAG, "三轴角速度为：X = %f, Y = %f, Z = %f", Data.Buf_Gyro[0], Data.Buf_Gyro[1], Data.Buf_Gyro[2]);
        ESP_LOGI(TAG, "三轴加速度为：Xg = %lf, Yg = %lf, Zg = %lf", Data.ACC_Gavity[0], Data.ACC_Gavity[1], Data.ACC_Gavity[2]);
        ESP_LOGI(TAG, "加速度偏角为：Angle_X = %lf, Angle_Y = %lf, Angle_Z = %lf", Data.ACC_Angle[0], Data.ACC_Angle[1], Data.ACC_Angle[2]);
        ESP_LOGI(TAG, "磁场偏角为：XoY = %f, XoZ = %f, YoZ = %f", Data.mag_Angle[0], Data.mag_Angle[1], Data.mag_Angle[2]);
        ESP_LOGI(TAG, "当前温度为：%.2lf ℃", (double)Data.temperature / 10.0);
        ESP_LOGI(TAG, "当前气压为：%ld Pa", Data.pressure);
        ESP_LOGI(TAG, "当前海拔为：%fm\n", 44330 * (1 - pow(Data.pressure / 101325.0, 1.0 / 5.255)));
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

/*************************数据读取**************************/
void ADXL345_Data_Update(void)
{
    ADXL345_Fifo_Read(Data.ACC_Gavity, 16);
    Data.ACC_Angle[0] = ADXL345_Angle(Data.ACC_Gavity, 1);
    Data.ACC_Angle[1] = ADXL345_Angle(Data.ACC_Gavity, 2);
    Data.ACC_Angle[2] = ADXL345_Angle(Data.ACC_Gavity, 0);
}

void L3G4200D_Data_Update(void)
{
    L3G4200D_Read_Average(Data.Buf_Gyro, 5);
}

void HMC5883L_Data_Update(void)
{
    HMC5883L_READ(Data.mag_Angle, Data.mag_XYZ, Data.Offset, Data.K_XYZ);
    // HMC5883L_Raw_Read(Data.mag_Angle, Data.mag_XYZ);
}

void BMP085_Data_Update(void)
{
    BMP085_Data_Calculate(&Data.temperature, &Data.pressure, Data.AC_123, Data.AC_456, Data.B1_MD);
}
