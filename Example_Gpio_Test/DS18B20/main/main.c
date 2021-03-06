#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#include "DS18B20.h"


void app_main(void)
{
    gpio_init();
    while (1)
    {
        temperature_convert();

        printf("当前温度为：%lf\n",temperature_read(0xBE));

        vTaskDelay(500 / portTICK_RATE_MS);

    }
    
    
}
