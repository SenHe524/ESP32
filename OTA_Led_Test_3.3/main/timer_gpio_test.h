#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#define GPIO_LED_IO 2 // LED
#define GPIO_LED_PIN_SEL (1ULL << GPIO_LED_IO)

#define GPIO_OUTPUT_IO_0 18 //蜂鸣器Gpio口
#define GPIO_OUTPUT_IO_1 19 //开关Gpio口
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0 22
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

#define TIMER_DIVIDER (16) //硬件定时器分频倍数Hardware timer clock divider
// 定时器计数频率convert counter value to seconds
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

// int flag_gpio_19 = 0; //开关i/o的标志位
//定时器相关参数结构体
typedef struct
{
    int timer_group;
    int timer_index;
    int alarm_interval;
    int auto_reload;
} timer_test_info_t;

//事件信息参数结构体
typedef struct
{
    timer_test_info_t timer_test_info;
    uint64_t timer_counter_val;
} timer_test_event_info_t;


// static xTaskCreate timer_task_t;//任务句柄

void timer_gpio_test(void);
void gpio_intr_init(void);                         // Gpio初始化

void Buzzer_control(timer_test_event_info_t evt);
void LED_init(void);


