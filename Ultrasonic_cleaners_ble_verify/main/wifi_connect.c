#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "wifi_connect.h"
#include "gap_gatts.h"
int wifi_flag = 1;
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

int wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();//创建一个事件组
    char wifi_name[30] = "TAC-GENRAY";
    char wifi_pw[30] = "tacgenray2022";
    uint32_t str_len = 30;
    esp_netif_create_default_wifi_sta();
	//定义一个名为cfg的wifi_init_config_t结构体，wifi_init_config_t的参数可由menuconfig配置
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	//初始名为cfg的结构体
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //定义一个叫instance_any_id的句柄
    esp_event_handler_instance_t instance_any_id;
    //定义一个叫instance_got_ip的句柄
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    nvs_open(NVS_DATA, NVS_READWRITE, &nvs_data_storage_handle);
    nvs_get_str(nvs_data_storage_handle, WIFI_SSID, wifi_name, &str_len);
    nvs_get_str(nvs_data_storage_handle, WIFI_PASSWORD, wifi_pw, &str_len);
    nvs_close(nvs_data_storage_handle);
    //初始化wifi的配置
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "test",
            .password = "testt",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    memcpy(wifi_config.sta.ssid,wifi_name, 30);
    memcpy(wifi_config.sta.password,wifi_pw, 30);
    //设置模式为站（station）模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    //初始化Station模式的Wifi配置
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    //启动wifi
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT){
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    }
    else if (bits & WIFI_FAIL_BIT){
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
                 wifi_flag = 0;
    }
    else{
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        wifi_flag = 0;
    }
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    return wifi_flag;
}