#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN 2

#define LED_HIGH gpio_set_level(LED_PIN,1);
#define LED_LOW gpio_set_level(LED_PIN,0);
#define LED_OUT_MODE gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT)
#define LED_INPUT_MODE gpio_set_direction(LED_PIN, GPIO_MODE_INPUT)


static TaskHandle_t Task_LED_t = NULL;
void Task_LED(void *parameter);


void Task_LED(void *parameter)
{
    while(1)
    {
        //printf("It works!");
        LED_OUT_MODE;
        LED_HIGH;
        vTaskDelay(1000 / portTICK_PERIOD_MS);//延时一秒
        LED_LOW;
        vTaskDelay(1000 / portTICK_PERIOD_MS);//延时一秒
    }
}

void app_main(void)
{
    xTaskCreate((TaskFunction_t)Task_LED,//任务入口函数
                                (const char* )"Task_LED",//任务名字
                                (uint16_t)512,//任务栈大小
                                (void* )NULL,//任务入口函数参数
                                (UBaseType_t )5,//任务优先级
                                (TaskHandle_t* )&Task_LED_t//任务句柄
                                );

}