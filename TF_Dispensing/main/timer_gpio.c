#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "timer_gpio.h"
#include "esp_task_wdt.h"

#include "gap_gatts.h"
#include "unistd.h"
static xQueueHandle gpio_Queue_t;  // Gpio队列句柄

static void timer_init_set(int group, int index, bool auto_reload, int timer_interval_sec); //定时器参数初始化与定时器注册函数
static bool IRAM_ATTR timer_isr_callback(void *args);                                   //定时器的ISR回调函数

static void IRAM_ATTR gpio_isr_handler(void *arg); // Gpio的ISR回调函数
static void gpio_task(void *arg);  
static void Buzzer_control_task(void *arg);

static int flag_timer_0 = 0; //定时器0的标志位
static int flag_timer_1 = 0; //定时器1的标志位
static int cnt_flag = 0; //定时器计数
static int flag_move = 0;   //物品放置标志位
// static int yellow_flag = 0;

uint32_t io_level_last = 1;
uint32_t io_level_now = 0;

uint16_t time_change_data = 2;
void gpio_intr_init(void);
void led_init(void);
void timer_gpio_init(void)
{
    gpio_intr_init();
    led_init();
    timer_init_set(TIMER_GROUP_0, TIMER_0, false, 5);
    timer_init_set(TIMER_GROUP_0, TIMER_1, false, 5);
    gpio_set_level(GPIO_Green_IO,1);
}

static bool IRAM_ATTR timer_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    timer_test_info_t *timer_info_callback = (timer_test_info_t *)args;
    uint64_t timer_counter_val = timer_group_get_counter_value_in_isr(
        timer_info_callback->timer_group,
        timer_info_callback->timer_index);

    if (!timer_info_callback->auto_reload)
    {
        timer_counter_val += timer_info_callback->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(
            timer_info_callback->timer_group,
            timer_info_callback->timer_index,
            timer_counter_val);
    }
    if(!timer_info_callback->timer_index)
    {    
        cnt_flag++;
        if(cnt_flag>=time_change_data)
        {
            timer_pause(TIMER_GROUP_0, TIMER_0); //停止0号定时器
            flag_timer_0 = 0;//0号定时器计时完成时，标志位置零
            if(io_level_last)
            {
                gpio_set_level(GPIO_Red_IO,0);
                gpio_set_level(GPIO_Green_IO,1);
                flag_move = 0;
            }
            else
            {
                timer_start(TIMER_GROUP_0, TIMER_1); //启动1号定时器
                flag_timer_1 = 1;//1号定时器开启，标志位置1

                gpio_set_level(GPIO_Yellow_IO,1);
                gpio_set_level(GPIO_Red_IO,0);
            }
            cnt_flag = 0;
        }
        else
        {
            timer_start(TIMER_GROUP_0, TIMER_0);
        }
    }
    else
    {
        timer_pause(TIMER_GROUP_0, TIMER_1); //停止1号定时器
        flag_timer_1 = 0;//1号定时器停止，标志位置0
    }
    return high_task_awoken == pdTRUE;
}

static void timer_init_set(int group, int index, bool auto_reload, int timer_interval_sec)
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
    timer_isr_callback_add(group, index, timer_isr_callback, timer_info, 0);
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
    io_conf.intr_type = GPIO_INTR_DISABLE;   // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;         // set as output mode
    io_conf.pin_bit_mask = GPIO_BUZZER_PIN_SEL; // bit mask of the pins that you want to set
    io_conf.pull_down_en = 0;                // disable pull-down mode
    io_conf.pull_up_en = 1;                  // disable pull-up mode
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_ANYEDGE;     // interrupt of rising edge and falling edge
    io_conf.pin_bit_mask = GPIO_SENSOR_PIN_SEL; // bit mask of the pins, use GPIO4/5 here
    io_conf.mode = GPIO_MODE_INPUT;            // set as input mode
    io_conf.pull_up_en = 1;                    // enable pull-up mode
    io_conf.pull_down_en = 0;                  // disable pull-down mode
    gpio_config(&io_conf);

    // change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_SENSOR_IO_0, GPIO_INTR_NEGEDGE);

    // create a queue to handle gpio event from isr
    gpio_Queue_t = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task, "gpio_task_example", 2048, NULL, 5, NULL);
    xTaskCreate(Buzzer_control_task,"Buzzer control",2048,NULL,10,NULL);
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //允许每个GPIO注册中断处理程序
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_SENSOR_IO_0, gpio_isr_handler, (void *)GPIO_SENSOR_IO_0);
}

static void gpio_task(void *arg)
{
    uint32_t io_num;
    for (;;)
    { //等待io口中断信息
        if (xQueueReceive(gpio_Queue_t, &io_num, portMAX_DELAY))
        {
            io_level_now = gpio_get_level(io_num);
            printf("GPIO[%d] intr, val: %d\n", io_num, io_level_now);

            if((!io_level_now) && io_level_last && (!flag_timer_0))// && (!yellow_flag)
            {
                flag_timer_0 = 1;
                flag_move = 1;

                //从NVS中读取定时器时间配置
                nvs_open(NVS_DATA, NVS_READWRITE, &nvs_data_storage_handle);
                nvs_get_u32(nvs_data_storage_handle, TIMER_CHANGE, &time_change_data);
                nvs_close(nvs_data_storage_handle);

                gpio_set_level(GPIO_Yellow_IO,0);
                gpio_set_level(GPIO_Red_IO,1);
                gpio_set_level(GPIO_Green_IO,0);
                timer_start(TIMER_GROUP_0, TIMER_0);
            }
            else if(io_level_now && (!io_level_last) && (!flag_timer_0))
            {
                gpio_set_level(GPIO_Yellow_IO,0);
                gpio_set_level(GPIO_Green_IO,1);
                flag_move = 0;
            }
            io_level_last = io_level_now;
        }
    }
}

static void Buzzer_control_task(void *arg)
{
    for (;;)
    {
        gpio_set_direction(GPIO_Buzzer_IO, GPIO_MODE_DEF_INPUT);
        while((!flag_timer_0) && flag_timer_1 && flag_move)
        {
            gpio_set_direction(GPIO_Buzzer_IO, GPIO_MODE_DEF_OUTPUT);
            gpio_set_level(GPIO_Buzzer_IO,0);
            usleep(100);
            gpio_set_level(GPIO_Buzzer_IO,1);
            usleep(100);

        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void led_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;   // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;         // set as output mode
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL; // bit mask of the pins that you want to set
    io_conf.pull_down_en = 1;                // enable pull-down mode
    io_conf.pull_up_en = 0;                  // disable pull-up mode
    gpio_config(&io_conf);
}