// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include "esp_timer.h"
// #include "esp_log.h"
// #include "sdkconfig.h"
// #include "esp_err.h"

#include <stdio.h>
#include "esp_timer.h"
#include "esp_log.h"
static const char *TAG = "TEST";
int i = 4;
void timer_test_callback(void *arg);
void app_main(void)
{
    esp_timer_init();//定时器接口初始化
    esp_timer_handle_t test_timer;//创建定时器句柄
    const esp_timer_create_args_t test_timer_args = {
            .callback = &timer_test_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = NULL,
            .name = "one-test_timer"
    };//定时器参数结构体
    // esp_timer_create(&test_timer_args, &test_timer); 
    // esp_timer_start_periodic(test_timer, 500 * 1000);
    
    //创建定时器
    ESP_ERROR_CHECK(esp_timer_create(&test_timer_args, &test_timer));
    //启动定时器
    ESP_ERROR_CHECK(esp_timer_start_periodic(test_timer, 500 * 1000));
    while (i);
    esp_timer_stop(test_timer);
    esp_timer_delete(test_timer);
    ESP_LOGI(TAG, "Stopped and deleted timers");
}
void timer_test_callback(void *arg)
{
    ESP_LOGI(TAG, "It works! %lldus\n", esp_timer_get_time());
    i--;
}