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
/***************************L3G4200D**********************************/
void app_main(void)
{
    if(!Init_L3G4200D()){
        printf("L3G4200D 初始化失败!\n");
    }
    while(1)
    {
        // if(HMC5883L_READ(BUF_HMC5883L, &Angle))
        // {
        //     printf("当前磁场角度为：%f\n",Angle);
        // }
        // else{
        //     printf("测量失败！\n");
        // }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

///***************************HMC5883L**********************************/
// uint8_t BUF_HMC5883L[6];
// float Angle;
// void app_main(void)
// {
//     if(!Init_HMC5883L()){
//         printf("HMC5883L 初始化失败!\n");
//     }
//     while(1)
//     {
//         if(HMC5883L_READ(BUF_HMC5883L, &Angle))
//         {
//             printf("当前磁场角度为：%f\n",Angle);
//         }
//         else{
//             printf("测量失败！\n");
//         }
//         vTaskDelay(1000 / portTICK_RATE_MS);
//     }
// }


///***************************ADXL345**********************************/
// uint8_t BUF_ADXL345[8];//临时存储数据
// uint8_t BUF[8] = {0};//存储合成的数据
// int A_X, A_Y, A_Z;
// float Angle_X, Angle_Y, Angle_Z;
// float Xg, Yg, Zg;

//ADXL345
// void app_main(void)
// {
// if(ADXL345_AUTO_Adjust())
//     {
//         if (Init_ADXL345())
//         {
//             while (1)
//             {
//                 ADXL345_Read_Average(&A_X, &A_Y, &A_Z, 10);
//                 Xg = ((A_X * 3.9) / 1000) * 9.8;
//                 Yg = ((A_Y * 3.9) / 1000) * 9.8;
//                 Zg = ((A_Z * 3.9) / 1000) * 9.8;
//                 Angle_X = ADXL345_Angle((float)A_X, (float)A_Y, (float)A_Z, 1);
//                 Angle_Y = ADXL345_Angle((float)A_X, (float)A_Y, (float)A_Z, 2);
//                 Angle_Z = ADXL345_Angle((float)A_X, (float)A_Y, (float)A_Z, 0);
//                 printf("Xg = %lf\n", Xg);
//                 printf("Yg = %lf\n", Yg);
//                 printf("Zg = %lf\n", Zg);
//                 printf("Angle_X = %lf\n", Angle_X);
//                 printf("Angle_Y = %lf\n", Angle_Y);
//                 printf("Angle_Z = %lf\n", Angle_Z);
//                 printf("\n");
//                 vTaskDelay(1000 / portTICK_RATE_MS);
//             }
//         }
//     }
// }


// /***************************BMP085**********************************/
// short AC_123[3];
// unsigned short AC_456[3];
// short B1_MD[5];
// long temperature, pressure;

// void app_main(void)
// {
//     Init_BMP085(AC_123, AC_456, B1_MD);
//     while(1)
//     {
//         if(BMP085_DATA(&temperature, &pressure, AC_123, AC_456, B1_MD)){
//             printf("当前温度为：%lf ℃\n", (double)temperature/10.0);
//             printf("当前压强为：%ld Pa\n",pressure);
//         }
//         else{
//         printf("获取压强温度失败！\n");
//         }
//         vTaskDelay(1000 / portTICK_RATE_MS);
//     }
// }
