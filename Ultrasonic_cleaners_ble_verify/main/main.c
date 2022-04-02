
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"
#include "errno.h"
#include "esp_wifi.h"

#include "ota_test.h"
#include "timer_gpio.h"
#include "gap_gatts.h"

extern int ota_flag;

void app_main(void)
{
    ota_test();
    while(1)
    {
        if(ota_flag == 1)
        {
            timer_gpio_test();
            //启用蓝牙之前，将wifi停止掉
            esp_wifi_stop();
            esp_wifi_disconnect();
            esp_wifi_deinit();
            
            ble_control();
            break;
        }
        vTaskDelay(10);
    }
    
}