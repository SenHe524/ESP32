#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#define GPIO_OUTPUT_IO_0 18 //蜂鸣器Gpio口
#define GPIO_OUTPUT_IO_1 19 //开关Gpio口
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0 4
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

//事件信息参数结构体
typedef struct
{
    timer_test_info_t timer_test_info;
    uint64_t timer_counter_val;
} timer_test_event_info_t;

int flag_gpio_19 = 0; //开关i/o的标志位

static xQueueHandle timer_Queue_t; //定时器队列句柄
static xQueueHandle gpio_Queue_t;  // Gpio队列句柄
// static xTaskCreate timer_task_t;//任务句柄

static void timer_init_test(int group, int index, bool auto_reload, int timer_interval_sec); //定时器参数初始化与定时器注册函数
static bool IRAM_ATTR timer_isr_callback_test(void *args);                                   //定时器的ISR回调函数
static void timer_task_example(void *arg);

void gpio_intr_init(void);                         // Gpio初始化
static void IRAM_ATTR gpio_isr_handler(void *arg); // Gpio的ISR回调函数
static void gpio_task_example(void *arg);          // Gpio任务

void Buzzer_control(timer_test_event_info_t evt);

void app_main(void)
{
    timer_Queue_t = xQueueCreate(10, sizeof(timer_test_event_info_t));

    gpio_intr_init();
    timer_init_test(TIMER_GROUP_0, TIMER_0, false, 3);
    timer_init_test(TIMER_GROUP_0, TIMER_1, false, 5);
    xTaskCreate(timer_task_example, "timer_task_example", 2048, NULL, 10, NULL);
    int cnt = 0;
    while (1)
    {
        cnt++;
        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

static bool IRAM_ATTR timer_isr_callback_test(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    timer_test_info_t *timer_info_callback = (timer_test_info_t *)args;
    uint64_t timer_counter_val = timer_group_get_counter_value_in_isr(
        timer_info_callback->timer_group,
        timer_info_callback->timer_index);
    //信息传递
    timer_test_event_info_t evt = {
        .timer_test_info.timer_group = timer_info_callback->timer_group,
        .timer_test_info.timer_index = timer_info_callback->timer_index,
        .timer_test_info.alarm_interval = timer_info_callback->alarm_interval,
        .timer_test_info.auto_reload = timer_info_callback->auto_reload,
        .timer_counter_val = timer_counter_val,
    };

    if (!timer_info_callback->auto_reload)
    {
        timer_counter_val += timer_info_callback->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(
            timer_info_callback->timer_group,
            timer_info_callback->timer_index,
            timer_counter_val);
    }
    xQueueSendFromISR(timer_Queue_t, &evt, &high_task_awoken);

    if (evt.timer_test_info.timer_index)
    {
        timer_pause(TIMER_GROUP_0, TIMER_1); //停止本定时器
    }
    else
    {
        timer_pause(TIMER_GROUP_0, TIMER_0); //停止本定时器
    }

    return high_task_awoken == pdTRUE;
}

static void timer_task_example(void *arg)
{
    timer_test_event_info_t evt;
    uint64_t task_counter_value;
    for (;;)
    {
        if (xQueueReceive(timer_Queue_t, &evt, portMAX_DELAY))
        {

            Buzzer_control(evt);

            printf("\n");
            printf("/*************************/\n");

            printf("Timer Group[%d],Timer index[%d]alarm event!\n",
                   evt.timer_test_info.timer_group, evt.timer_test_info.timer_index);
            printf("Time  : %.8f s\r\n", (double)evt.timer_counter_val / TIMER_SCALE);
            timer_get_counter_value(evt.timer_test_info.timer_group,
                                    evt.timer_test_info.timer_index, &task_counter_value);
            printf("Time  : %.8f s\r\n", (double)task_counter_value / TIMER_SCALE);

            printf("/*************************/\n");
            printf("\n");
        }
    }
}

static void timer_init_test(int group, int index, bool auto_reload, int timer_interval_sec)
{
    timer_config_t timer_config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload}; //定时器参数结构体配置

    timer_init(group, index, &timer_config); //初始化定时器

    timer_set_counter_value(group, index, 0); //设置定时器初始值
    //设定定时器报警阈值
    timer_set_alarm_value(group, index, timer_interval_sec * TIMER_SCALE);
    //设置定时器中断使能
    timer_enable_intr(group, index);
    //定义一个定时器参数结构体，并将相关参数存入其中
    timer_test_info_t *timer_info = calloc(1, sizeof(timer_test_info_t));
    timer_info->timer_group = group;
    timer_info->timer_index = index;
    timer_info->alarm_interval = timer_interval_sec;
    timer_info->auto_reload = auto_reload;

    //注册ISR中断
    timer_isr_callback_add(group, index, timer_isr_callback_test, timer_info, 0);
    // timer_start(group,index);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    //检测io口中断
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_Queue_t, &gpio_num, NULL);
}

void gpio_intr_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;      // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;            // set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // bit mask of the pins that you want to set
    io_conf.pull_down_en = 0;                   // disable pull-down mode
    io_conf.pull_up_en = 0;                     // disable pull-up mode
    gpio_config(&io_conf);                      // configure GPIO with the given settings

    io_conf.intr_type = GPIO_INTR_POSEDGE;                     // interrupt of rising edge
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; // bit mask of the pins, use GPIO4/5 here
    io_conf.mode = GPIO_MODE_INPUT;            // set as input mode
    io_conf.pull_up_en = 0;                    // enable pull-up mode
    io_conf.pull_down_en = 1;                  // disable pull-down mode
    gpio_config(&io_conf);

    // change gpio intrrupt type for one pin
    // gpio_set_intr_type(GPIO_INPUT_IO_0, 2);

    // create a queue to handle gpio event from isr
    gpio_Queue_t = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //允许每个GPIO注册中断处理程序
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    { //等待io口中断信息
        if (xQueueReceive(gpio_Queue_t, &io_num, portMAX_DELAY))
        {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));

            //开关i/o的标志位flag_gpio_19置1

            timer_start(TIMER_GROUP_0, TIMER_0);
            timer_start(TIMER_GROUP_0, TIMER_1);

            flag_gpio_19 = 1;
        }
    }
}

void Buzzer_control(timer_test_event_info_t evt)
{
    if (!evt.timer_test_info.timer_index)
    {
        printf("Buezzer!\n");
    }
}
