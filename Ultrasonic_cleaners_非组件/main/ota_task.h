#ifndef _OTA_TASK_H_
#define _OTA_TASK_H_

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

#define BUFFSIZE 1024

#define OTA_URL_SIZE 256

xQueueHandle ota_Queue_t;

void ota_detection(void);

#endif
