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
#include "ota_task.h"
#include "wifi_connect.h"
#include "gap_gatts.h"
int ota_flag = 0;
static char ota_url[64] ={"http://172.18.0.70:10031/motoc/gImage/UC1000.bin"};
xQueueHandle ota_Queue_t;

static const char *TAG = "Ultrasonic_cleaners_Ota";
/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = {0};
static void http_cleanup(esp_http_client_handle_t client);
static void __attribute__((noreturn)) task_fatal_error(void);
static void ota_task(void *pvParameter);
static bool diagnostic(void);
static TaskHandle_t Task_Ota_t = NULL;

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static void __attribute__((noreturn)) task_fatal_error(void)
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    ota_flag = 1;
    xQueueSend(ota_Queue_t, &ota_flag, NULL);
    (void)vTaskDelete(NULL);

    while (1)
    {
        ;
    }
}

//OTA任务
static void ota_task(void *pvParameter)
{
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA example");

    //获取当前配置的启动应用程序的分区信息
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    //获取当前运行的启动应用程序的分区信息
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);
    uint32_t str_len_1 = 64;
    nvs_open(NVS_DATA, NVS_READWRITE, &nvs_data_storage_handle);
    nvs_get_str(nvs_data_storage_handle, OTA_URL, ota_url, &str_len_1);
    nvs_close(nvs_data_storage_handle);
    esp_http_client_config_t config = {
        // .url = ota_url,
        .url = CONFIG_EXAMPLE_FIRMWARE_UPG_URL,
        .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
    };

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
    char url_buf[OTA_URL_SIZE];
    if (strcmp(config.url, "FROM_STDIN") == 0)
    {
        example_configure_stdin_stdout();
        fgets(url_buf, OTA_URL_SIZE, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0';
        config.url = url_buf;
    }
    else
    {
        ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
        abort();
    }
#endif

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        task_fatal_error();
    }
    esp_http_client_set_url(client, ota_url);
    //连http服务器
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    esp_http_client_fetch_headers(client);

    //获取紧邻 当前使用的OTA_X分区 的下一个可用于烧录升级固件的Flash分区
    //若当前为OTA_0,则获取到的为OTA_1，若当前为OTA_1,则获取到的为OTA_0，
    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);

    int binary_file_length = 0;
    /*deal with all receive packet*/
    bool image_header_was_checked = false;
    while (1)
    {
        //从Http读取数据
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0)
        {
            ESP_LOGE(TAG, "Error: SSL data read error");
            http_cleanup(client);
            esp_ota_abort(update_handle);
            task_fatal_error();
        }
        else if (data_read > 0)
        {
            if (image_header_was_checked == false)
            {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    // check current version with downloading
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL)
                    {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            http_cleanup(client);
                            esp_ota_abort(update_handle);
                            task_fatal_error();
                        }
                    }
#ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) <= 0)
                    {
                        ESP_LOGW(TAG, "Current running version is the same as the new or later. We will not continue the update.");
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
#endif

                    image_header_was_checked = true;

                    // OTA开始写（做写前准备）
                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");
                }
                else
                {
                    ESP_LOGE(TAG, "received package is not fit len");
                    http_cleanup(client);
                    esp_ota_abort(update_handle);
                    task_fatal_error();
                }
            }
            //写入falsh
            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK)
            {
                http_cleanup(client);
                esp_ota_abort(update_handle);
                task_fatal_error();
            }
            binary_file_length += data_read;
            ESP_LOGD(TAG, "Written image length %d", binary_file_length);
        }
        else if (data_read == 0)
        {
            if (errno == ECONNRESET || errno == ENOTCONN)
            {
                ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true)
            {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
        }
    }
    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true)
    {
        ESP_LOGE(TAG, "Error in receiving complete file");
        http_cleanup(client);
        esp_ota_abort(update_handle);
        task_fatal_error();
    }

    // OTA写结束
    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        else
        {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        http_cleanup(client);
        task_fatal_error();
    }

    // OTA data区数据升级完成，重启并获取OTA data区数据，以决定此后加载哪一个分区的固件执行
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    ota_flag = 1;
    xQueueSend(ota_Queue_t, &ota_flag, NULL);
    esp_restart();
    return ;
}

static bool diagnostic(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Diagnostics (5 sec)...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    bool diagnostic_is_ok = gpio_get_level(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);

    gpio_reset_pin(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

void ota_detection(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            // run diagnostic function ...
            bool diagnostic_is_ok = diagnostic();
            if (diagnostic_is_ok)
            {
                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
            }
            else
            {
                ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    //创建LwIP核心任务并初始化相关工作（初始化底层TCP/IP栈。）
    ESP_ERROR_CHECK(esp_netif_init());
    // 创建系统事件任务并初始化应用程序事件的回调函数
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ota_Queue_t = xQueueCreate(10, sizeof(int));//创建OTA队列句柄
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

#if CONFIG_EXAMPLE_CONNECT_WIFI
    esp_wifi_set_ps(WIFI_PS_NONE);//设置wifi不节能
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
    if(wifi_init_sta()){//若wifi初始化及连接成功，则创建OTA任务
        xTaskCreate(&ota_task, "ota_task", 4096, NULL, 4, (TaskHandle_t *)&Task_Ota_t);
    }
    else{//否则跳过OTA
        ota_flag = 1;
        xQueueSend(ota_Queue_t, &ota_flag, NULL);
    }
}
