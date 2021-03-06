#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#define GATTS_TAG "GATTS"

#define DEVICE_NAME            "TF_Dispensing"


#define GATTS_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)


#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define SVC_INST_ID 0

static const char *NVS_DATA = "nvs_data";
static const char *TIMER_CHANGE = "timer_change";
static const char *WIFI_SSID = "wifi_ssid";
static const char *WIFI_PASSWORD = "wifi_password";
static const char *OTA_URL = "ota_url";
static const char *PASSKEY = "passkey";
nvs_handle_t nvs_data_storage_handle;  //NVS存储区句柄

enum
{
    IDX_SVC,
    
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    IDX_CHAR_D,
    IDX_CHAR_VAL_D,

    IDX_CHAR_E,
    IDX_CHAR_VAL_E,

    IDX_CHAR_F,
    IDX_CHAR_VAL_F,

    IDX_CHAR_G,
    IDX_CHAR_VAL_G,

    IDX_CHAR_H,
    IDX_CHAR_VAL_H,

    IDX_CHAR_I,
    IDX_CHAR_VAL_I,

    IDX_NB,
};



void ble_control(void);