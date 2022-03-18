#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"


#define GPIO_Green_IO 14 // 绿指示灯io口
#define GPIO_Red_IO 12 // 红色指示灯io口
#define GPIO_Yellow_IO 13 // 黄色指示灯io口
#define GPIO_LED_PIN_SEL ((1ULL << GPIO_Green_IO) | (1ULL << GPIO_Red_IO) | (1ULL << GPIO_Yellow_IO))


#define GPIO_Buzzer_IO 18 // Buzzerio口
#define GPIO_Buzzer_PIN_SEL (1ULL << GPIO_Buzzer_IO)

#define GPIO_OUTPUT_IO_1 19 //开关Gpio口
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_1)

#define GPIO_INPUT_IO_0 22  //传感器io口
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_0)

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


void timer_gpio_test(void);
void gpio_intr_init(void);
void LED_init(void);

