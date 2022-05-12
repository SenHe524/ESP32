#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#include "DHT11.h"


void app_main(void)
{
    //上电延时2s等待DHT11做准备
    vTaskDelay(2000 / portTICK_RATE_MS);
    gpio_init();

    while (1)
    {
        data_read();
        vTaskDelay(500 / portTICK_RATE_MS);
    }
    
    
}
