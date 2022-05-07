#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#include "DS18B20.h"

#define DS18B20_SKIP_ROM         0xCC
#define DS18B20_CONVERT_T        0x44
#define DS18B20_READ_SCRATCHPAD  0xBE


void app_main(void)
{
    gpio_init();
    while (1)
    {
        uint8_t T_LOW, T_HIGH;
        int TEMP;
        float T;

        while(OneWire_Init()) {
            printf("正在初始化...\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        OneWire_SendByte(DS18B20_SKIP_ROM);//跳过ROM
        OneWire_SendByte(DS18B20_CONVERT_T);//让DS18B20将温度转换为我们所需要的数值

        ets_delay_us(750 * 1000);//延时750ms，以待DS18B20将温度转换完成

        while(OneWire_Init()) {
            printf("正在初始化...\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        OneWire_SendByte(DS18B20_SKIP_ROM);//跳过ROM
        OneWire_SendByte(DS18B20_READ_SCRATCHPAD);//读取温度寄存器
        T_LOW = OneWire_ReceiveByte();
        T_HIGH = OneWire_ReceiveByte();
        TEMP = (T_HIGH << 8) | T_LOW;
        T = TEMP/16.0;
        // T = (float)(T_LOW + (T_HIGH * 256)) / 16;
        printf("当前温度为：%lf\n",T);

        vTaskDelay(500 / portTICK_RATE_MS);

    }
    
    
}
