#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN 2

#define LED_HIGH gpio_set_level(LED_PIN,1);
#define LED_LOW gpio_set_level(LED_PIN,0);
#define LED_OUT_MODE gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT)
#define LED_INPUT_MODE gpio_set_direction(LED_PIN, GPIO_MODE_INPUT)

void app_main(void)
{

    while(1)
    {
        printf("It works!");
        LED_OUT_MODE;
        LED_HIGH;
        vTaskDelay(1000 / portTICK_PERIOD_MS);//延时一秒
        LED_LOW;
        vTaskDelay(1000 / portTICK_PERIOD_MS);//延时一秒
    }

}