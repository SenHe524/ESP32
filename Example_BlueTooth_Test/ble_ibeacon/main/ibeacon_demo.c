// #include <stdint.h>
// #include <string.h>
// #include <stdbool.h>
// #include <stdio.h>
// #include "nvs_flash.h"

// #include "esp_bt.h"
// #include "esp_gap_ble_api.h"
// #include "esp_gattc_api.h"
// #include "esp_gatt_defs.h"
// #include "esp_bt_main.h"
// #include "esp_bt_defs.h"
// #include "esp_ibeacon_api.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"

// static const char* DEMO_TAG = "IBEACON_DEMO";
// extern esp_ble_ibeacon_vendor_t vendor_config;

// ///Declare static functions
// static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// static esp_ble_adv_params_t ble_adv_params = {
//     .adv_int_min        = 0x20,
//     .adv_int_max        = 0x40,
//     .adv_type           = ADV_TYPE_IND,
//     .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
//     .channel_map        = ADV_CHNL_ALL,
//     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };

// static uint8_t adv_service_uuid128[32] = {
//     /* LSB <--------------------------------------------------------------------------------> MSB */
//     //first uuid, 16bit, [12],[13] is the value
//     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
//     //second uuid, 32bit, [12], [13], [14], [15] is the value
//     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
// };

// static uint8_t user_data1[3] = {0x11, 0x22, 0x33};
// static uint8_t user_data2[3] = {0x66, 0x77, 0x88};

// static esp_ble_adv_data_t srsp_data = {
//     .set_scan_rsp = true,
//     .manufacturer_len = sizeof(user_data2),
//     .p_manufacturer_data = user_data2,
// };

// static esp_ble_adv_data_t adv_data = {
//     .set_scan_rsp = false,
//     .include_name = true,
//     .min_interval = 0x000A, //slave connection min interval, Time = min_interval * 1.25 msec=7.5ms
//     .max_interval = 0x0014, //slave connection max interval, Time = max_interval * 1.25 msec=20ms
//     .appearance = ESP_BLE_APPEARANCE_GENERIC_PHONE,
//     .manufacturer_len = sizeof(user_data1),
//     .p_manufacturer_data = user_data1,
//     .service_uuid_len = sizeof(adv_service_uuid128),
//     .p_service_uuid = adv_service_uuid128,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };


// static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
// {
//     esp_err_t err;

//     switch (event) {
//     case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:{
//         esp_ble_gap_start_advertising(&ble_adv_params);
//         break;
//     }
//     case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
//         //adv start complete event to indicate adv start successfully or failed
//         if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
//             ESP_LOGE(DEMO_TAG, "Adv start failed: %s", esp_err_to_name(err));
//         }
//         break;
//     case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
//         if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
//             ESP_LOGE(DEMO_TAG, "Adv stop failed: %s", esp_err_to_name(err));
//         }
//         else {
//             ESP_LOGI(DEMO_TAG, "Stop adv successfully");
//         }
//         break;

//     default:
//         break;
//     }
// }

// void ble_ibeacon_appRegister(void)
// {
//     esp_err_t status;
//     ESP_LOGI(DEMO_TAG, "register callback");
//     //register the scan callback function to the gap module
//     if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
//         ESP_LOGE(DEMO_TAG, "gap register error: %s", esp_err_to_name(status));
//         return;
//     }
// }

// void ble_ibeacon_init(void)
// {
//     esp_bluedroid_init();
//     esp_bluedroid_enable();
//     ble_ibeacon_appRegister();
// }

// void app_main(void)
// {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     esp_bt_controller_init(&bt_cfg);
//     esp_bt_controller_enable(ESP_BT_MODE_BLE);

//     ble_ibeacon_init();

//     esp_ble_gap_set_device_name("ESP_TEST");
//     esp_ble_gap_config_adv_data(&srsp_data);
//     esp_ble_gap_config_adv_data(&adv_data);
// }

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_ibeacon_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char* DEMO_TAG = "IBEACON_DEMO";

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static uint8_t user_data[3] = {0x11, 0x22, 0x33};
static uint8_t user_data2[3] = {0x66, 0x77, 0x88};

static esp_ble_adv_data_t srsp_data = {
    .set_scan_rsp = true,
    .manufacturer_len = sizeof(user_data2),
    .p_manufacturer_data = user_data2,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .min_interval = 0x000A, //slave connection min interval, Time = min_interval * 1.25 msec=7.5ms
    .max_interval = 0x0014, //slave connection max interval, Time = max_interval * 1.25 msec=20ms
    .appearance = ESP_BLE_APPEARANCE_GENERIC_PHONE,
    .manufacturer_len = sizeof(user_data),
    .p_manufacturer_data = user_data,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
        
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:{
        esp_ble_gap_start_advertising(&ble_adv_params);
        break;
    }

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //adv start complete event to indicate adv start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Adv start failed: %s", esp_err_to_name(err));
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(DEMO_TAG, "Stop adv successfully");
        }
        break;

    default:
        break;
    }
}


void ble_ibeacon_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG, "gap register error: %s", esp_err_to_name(status));
        return;
    }

}

void ble_ibeacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_ibeacon_appRegister();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    ble_ibeacon_init();

    esp_ble_gap_set_device_name("hello");
    esp_ble_gap_config_adv_data(&srsp_data);
    esp_ble_gap_config_adv_data(&adv_data);
}