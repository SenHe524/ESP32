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
#include "Init_Read.h"
#include "Register.h"
/******************************变量定义*********************************/
typedef struct
{
    //L3G4200D
    int Init_K3G4200D; //初始化标志位
    float BUF_L3G4200D[3];//L3G4200D
    float BUF_MARK[3];
    //HMC5883L
    uint8_t BUF_HMC5883L[6];
    float Angle;
    // //BMP0885
    short AC_123[3];
    unsigned short AC_456[3];
    short B1_MD[5];
    long temperature, pressure;
    //ADXL345
    int A_X, A_Y, A_Z;
    float Angle_X, Angle_Y, Angle_Z;
    float Xg, Yg, Zg;
    
}  GY_80_DATA_t;

/*************************任务句柄及回调函数声明**************************/
static TaskHandle_t Task_GY_80_t = NULL;
void Task_GY_80(GY_80_DATA_t *parameter);
/*************************数据读取函数声明**************************/
void Init_GY_80(GY_80_DATA_t *parameter);
void L3G4200D_DATA(GY_80_DATA_t *parameter);
void ADXL345_DATA(GY_80_DATA_t *parameter);
void HMC5883L_DATA(GY_80_DATA_t *parameter);
void BMP085_DATA(GY_80_DATA_t *parameter);

/******************************主函数*********************************/
void app_main(void)
{
    GY_80_DATA_t Data = {
        .Init_K3G4200D = 0,
        .BUF_MARK[0] = 0,
        .BUF_MARK[1] = 0,
        .BUF_MARK[2] = 0,
    };
    // xTaskCreate((TaskFunction_t)Task_L3G4200D,"Task_L3G4200D",2048,NULL,5,&Task_L3G4200D_t);
    // xTaskCreate((TaskFunction_t)Task_HMC5883L,"Task_HMC5883L",2048,NULL,5,&Task_HMC5883L_t);
    // xTaskCreate((TaskFunction_t)Task_BMP085,"Task_BMP085",2048,NULL,5,&Task_BMP085_t);
    // xTaskCreate((TaskFunction_t)Task_ADXL345,"Task_ADXL345",2048,&Data,5,&Task_ADXL345_t);
    xTaskCreate((TaskFunction_t)Task_GY_80,"Task_GY_80",8192,&Data,5,&Task_GY_80_t);
}
/***************************Init GY_80*********************************/
void Init_GY_80(GY_80_DATA_t *parameter)
{
    GY_80_DATA_t *Data = parameter;
    //初始化ADXL345
    while(!Init_ADXL345());
    while(!ADXL345_AUTO_Adjust());
    while(!Init_ADXL345());
    //初始化L3G4200D
    while(!Data->Init_K3G4200D){
        if(!Init_L3G4200D(Data->BUF_MARK)){
                printf("L3G4200D 初始化失败!\n");
                Data->Init_K3G4200D = 0;
            }
            else
            {
                if((Data->BUF_MARK[0] > 1) || (Data->BUF_MARK[1] > 1) || (Data->BUF_MARK[2] > 1)){
                    printf("L3G4200D 初始化失败!\n");
                    Data->Init_K3G4200D = 0;
                }
                else{
                    Data->Init_K3G4200D = 1;
                }
                // printf("%f,%f,%f\t1000\n",BUF_MARK[0],BUF_MARK[1],BUF_MARK[2]);
            }
    }
    //初始化HMC5883L
    while(!Init_HMC5883L()){
        printf("HMC5883L 初始化失败!\n");
    }
    //初始化BMP085
    Init_BMP085(Data->AC_123, Data->AC_456, Data->B1_MD);
}
/***************************GY_80*********************************/
void Task_GY_80(GY_80_DATA_t *parameter)
{
    GY_80_DATA_t *Data = parameter;
    Init_GY_80(Data);
    while (1)
    {
        /*****************读取加速度*******************/
        ADXL345_DATA(Data);
        /*****************读取角速度*******************/
        L3G4200D_DATA(Data);
        /*****************读取温度和压强*******************/
        BMP085_DATA(Data);
        /*****************读取磁场角度*******************/
        HMC5883L_DATA(Data);
        printf("\n\n");
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

/*************************数据读取函数**************************/
void L3G4200D_DATA(GY_80_DATA_t *parameter)
{
    GY_80_DATA_t *Data = parameter;
    if((L3G4200D_READ_AVERAGE(Data->BUF_L3G4200D, Data->BUF_MARK, 5) ==1) && 
            (Data->Init_K3G4200D == 1))
    {
        printf("三轴角速度为：X = %f, Y = %f, Z = %f\n", Data->BUF_L3G4200D[0], Data->BUF_L3G4200D[1],Data->BUF_L3G4200D[2]);
    }
    else{
        if(!Init_L3G4200D(Data->BUF_MARK)){
            printf("L3G4200D 初始化失败!\n");
            Data->Init_K3G4200D = 0;
        }
        else
        {
            if((Data->BUF_MARK[0] > 1) || (Data->BUF_MARK[1] > 1) || (Data->BUF_MARK[2] > 1)){
                printf("L3G4200D 初始化失败!\n");
                Data->Init_K3G4200D = 0;
            }
            else{
                Data->Init_K3G4200D = 1;
            }
            // printf("%f,%f,%f\t1000\n",BUF_MARK[0],BUF_MARK[1],BUF_MARK[2]);
        }
    }
}
void ADXL345_DATA(GY_80_DATA_t *parameter)
{
    GY_80_DATA_t *Data = parameter;
    ADXL345_Read_Average(&Data->A_X, &Data->A_Y, &Data->A_Z, 10);
    Data->Xg = (((float)Data->A_X * 3.9) / 1000) * 9.8;
    Data->Yg = (((float)Data->A_Y * 3.9) / 1000) * 9.8;
    Data->Zg = (((float)Data->A_Z * 3.9) / 1000) * 9.8;
    Data->Angle_X = ADXL345_Angle((float)Data->A_X * 3.9, (float)Data->A_Y * 3.9, (float)Data->A_Z * 3.9, 1);
    Data->Angle_Y = ADXL345_Angle((float)Data->A_X * 3.9, (float)Data->A_Y * 3.9, (float)Data->A_Z * 3.9, 2);
    Data->Angle_Z = ADXL345_Angle((float)Data->A_X * 3.9, (float)Data->A_Y * 3.9, (float)Data->A_Z * 3.9, 0);
    printf("三轴加速度为：Xg = %lf, Yg = %lf, Zg = %lf\n", Data->Xg, Data->Yg, Data->Zg);
    printf("偏角为：Angle_X = %lf, Angle_Y = %lf, Angle_Z = %lf\n", Data->Angle_X, Data->Angle_Y, Data->Angle_Z);
}
void HMC5883L_DATA(GY_80_DATA_t *parameter)
{
    GY_80_DATA_t *Data = parameter;
    HMC5883L_READ(Data->BUF_HMC5883L, &Data->Angle);
    printf("当前磁场角度为：%f\n",Data->Angle);

}
void BMP085_DATA(GY_80_DATA_t *parameter)
{
    GY_80_DATA_t *Data = parameter;
    if(BMP085_DATA_CALCULATE(&Data->temperature, &Data->pressure, Data->AC_123, Data->AC_456, Data->B1_MD)){
        printf("当前温度为：%.2lf ℃\n", (double)Data->temperature/10.0);
        printf("当前压强为：%ld Pa\n",Data->pressure);
    }
    else{
    printf("获取压强温度失败！\n");
    }
}
