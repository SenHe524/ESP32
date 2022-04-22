#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"


#define GPIO_Green_IO 14 // 绿指示灯io口
#define GPIO_Red_IO 12 // 红色指示灯io口
#define GPIO_Yellow_IO 13 // 黄色指示灯io口
#define GPIO_wifi_IO 27 // wifi指示灯io口
#define GPIO_LED_PIN_SEL ((1ULL << GPIO_Green_IO) | (1ULL << GPIO_Red_IO) | (1ULL << GPIO_Yellow_IO) | (1ULL << GPIO_wifi_IO))

#define GPIO_Buzzer_IO 26 // 蜂鸣器io口
#define GPIO_BUZZER_PIN_SEL (1ULL << GPIO_Buzzer_IO)


#define GPIO_SWITCH_OPEN_IO 18 //开Gpio口
#define GPIO_SWITCH_CLOSE_IO 19 //关Gpio口
#define GPIO_SWITCH_PIN_SEL ((1ULL << GPIO_SWITCH_OPEN_IO) | (1ULL << GPIO_SWITCH_CLOSE_IO))

#define GPIO_SENSOR_IO_0 22  //传感器io口
#define GPIO_SENSOR_PIN_SEL (1ULL << GPIO_SENSOR_IO_0)

#define ESP_INTR_FLAG_DEFAULT 0

#define TIMER_DIVIDER (16) //硬件定时器分频倍数Hardware timer clock divider
// 定时器计数频率convert counter value to seconds
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)


//定时器相关参数结构体
typedef struct
{
    int timer_group;
    int timer_index;
    int alarm_interval;
    int auto_reload;
} timer_test_info_t;


void timer_gpio_init(void);

