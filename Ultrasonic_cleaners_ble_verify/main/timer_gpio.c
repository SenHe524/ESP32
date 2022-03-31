#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "timer_gpio.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

static xQueueHandle gpio_Queue_t;  // Gpio队列句柄

static void timer_init_test(int group, int index, bool auto_reload, int timer_interval_sec); //定时器参数初始化与定时器注册函数
static bool IRAM_ATTR timer_isr_callback_test(void *args);                                   //定时器的ISR回调函数
void soft_timer_callback_open(void *arg);
void soft_timer_callback_close(void *arg);

static void IRAM_ATTR gpio_isr_handler(void *arg); // Gpio的ISR回调函数
static void gpio_task(void *arg);  
void LED_PWM_init(void);

static int flag_timer_0 = 0; //定时器0的标志位
static int flag_timer_1 = 0; //定时器1的标志位

uint16_t time_change_flag = 510;
uint8_t timer_change_0 = 0;
uint8_t timer_change_1 = 0;

esp_timer_handle_t soft_timer_open;//创建软件定时器句柄
const esp_timer_create_args_t soft_timer_args_open = {
            .callback = &soft_timer_callback_open,
            .arg = NULL,
            .name = "soft_timer_open"
    };
esp_timer_handle_t soft_timer_close;//创建软件定时器句柄
const esp_timer_create_args_t soft_timer_args_close = {
            .callback = &soft_timer_callback_close,
            .arg = NULL,
            .name = "soft_timer_close"
    };

void timer_gpio_test(void)
{
    esp_timer_init();    //软件定时器接口初始化
    gpio_intr_init();
    LED_PWM_init();
    gpio_set_level(GPIO_Green_IO,1);
    timer_init_test(TIMER_GROUP_0, TIMER_0, true,5);
    timer_init_test(TIMER_GROUP_0, TIMER_1, true, 10);
}

static bool IRAM_ATTR timer_isr_callback_test(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    timer_test_info_t *timer_info_callback = (timer_test_info_t *)args;
    uint64_t timer_counter_val = timer_group_get_counter_value_in_isr(
        timer_info_callback->timer_group,
        timer_info_callback->timer_index);
    timer_change_0 = time_change_flag/100;
    timer_change_1 = time_change_flag%100;
    if (!timer_info_callback->auto_reload)
    {
        if (!timer_info_callback->timer_index)
        {
            timer_counter_val += timer_change_0 * TIMER_SCALE;
            timer_group_set_alarm_value_in_isr(
                timer_info_callback->timer_group,
                timer_info_callback->timer_index,
                timer_counter_val);
        }
        else
        {
            timer_counter_val += timer_change_1 * TIMER_SCALE;
            timer_group_set_alarm_value_in_isr(
                timer_info_callback->timer_group,
                timer_info_callback->timer_index,
                timer_counter_val);
        }
    }
    else
    {
         if (!timer_info_callback->timer_index)
        {
            timer_counter_val = timer_change_0 * TIMER_SCALE;
            timer_group_set_alarm_value_in_isr(
                timer_info_callback->timer_group,
                timer_info_callback->timer_index,
                timer_counter_val);
        }
        else
        {
            timer_counter_val = timer_change_1 * TIMER_SCALE;
            timer_group_set_alarm_value_in_isr(
                timer_info_callback->timer_group,
                timer_info_callback->timer_index,
                timer_counter_val);
        }
    }

    if (timer_info_callback->timer_index)
    {
        timer_pause(TIMER_GROUP_0, TIMER_1); //停止本定时器
        flag_timer_1 = 0;   //1号定时器计时完成时，标志位置零
        
        gpio_set_level(GPIO_Green_IO,1);
        gpio_set_level(GPIO_Yellow_IO,0);
        //启动软件定时器计时500ms，并将GPIO口19电平拉高，在10ms后拉低模拟脉冲
        esp_timer_create(&soft_timer_args_close, &soft_timer_close);
        esp_timer_start_once(soft_timer_close, 10 * 1000);
        gpio_set_direction(GPIO_SWITCH_CLOSE_IO,GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_SWITCH_CLOSE_IO,1);
    }
    else
    {
        timer_pause(TIMER_GROUP_0, TIMER_0); //停止本定时器
        flag_timer_0 = 0;//0号定时器计时完成时，标志位置零

        gpio_set_level(GPIO_Yellow_IO,1);
        gpio_set_level(GPIO_Red_IO,0);
    }

    return high_task_awoken == pdTRUE;
}

void soft_timer_callback_open(void *arg)
{
    gpio_set_level(GPIO_SWITCH_OPEN_IO,0);
    gpio_set_direction(GPIO_SWITCH_OPEN_IO,GPIO_MODE_INPUT);
    esp_timer_stop(soft_timer_open);
    esp_timer_delete(soft_timer_open);
}
void soft_timer_callback_close(void *arg)
{
    gpio_set_level(GPIO_SWITCH_CLOSE_IO,0);
    gpio_set_direction(GPIO_SWITCH_CLOSE_IO,GPIO_MODE_INPUT);
    esp_timer_stop(soft_timer_close);
    esp_timer_delete(soft_timer_close);
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
    // io_conf.intr_type = GPIO_INTR_DISABLE;   // disable interrupt
    // io_conf.mode = GPIO_MODE_INPUT;         // set as output mode
    // io_conf.pin_bit_mask = GPIO_Buzzer_PIN_SEL; // bit mask of the pins that you want to set
    // io_conf.pull_down_en = 0;                // disable pull-down mode
    // io_conf.pull_up_en = 0;                  // disable pull-up mode
    // gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;     // interrupt of rising edge
    io_conf.pin_bit_mask = GPIO_SENSOR_PIN_SEL; // bit mask of the pins, use GPIO4/5 here
    io_conf.mode = GPIO_MODE_INPUT;            // set as input mode
    io_conf.pull_up_en = 0;                    // enable pull-up mode
    io_conf.pull_down_en = 0;                  // disable pull-down mode
    gpio_config(&io_conf);

    // change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_SENSOR_IO_0, GPIO_INTR_NEGEDGE);

    // create a queue to handle gpio event from isr
    gpio_Queue_t = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task, "gpio_task_example", 2048, NULL, 5, NULL);
    // xTaskCreate(Buzzer_control_task,"Buzzer control",2048,NULL,10,NULL);
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //允许每个GPIO注册中断处理程序
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_SENSOR_IO_0, gpio_isr_handler, (void *)GPIO_SENSOR_IO_0);
}

static void gpio_task(void *arg)
{
    uint32_t io_num;
    uint32_t io_level;
    for (;;)
    { //等待io口中断信息
        if (xQueueReceive(gpio_Queue_t, &io_num, portMAX_DELAY))
        {
            io_level = gpio_get_level(io_num);
            printf("GPIO[%d] intr, val: %d\n", io_num, io_level);

            if(!io_level && !flag_timer_0 && !flag_timer_1)
            {
                flag_timer_0 = 1;
                flag_timer_1 = 1;

                gpio_set_level(GPIO_Red_IO,1);
                gpio_set_level(GPIO_Green_IO,0);
                
                //启动软件定时器计时500ms，并将GPIO口18电平拉高，在200ms后拉低模拟脉冲
                esp_timer_create(&soft_timer_args_open, &soft_timer_open);
                esp_timer_start_once(soft_timer_open, 200 * 1000);
                gpio_set_direction(GPIO_SWITCH_OPEN_IO,GPIO_MODE_OUTPUT);
                gpio_set_level(GPIO_SWITCH_OPEN_IO,1);

                timer_start(TIMER_GROUP_0, TIMER_0);
                timer_start(TIMER_GROUP_0, TIMER_1);
            }
        }
    }
}

void LED_PWM_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;   // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;         // set as output mode
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL; // bit mask of the pins that you want to set
    io_conf.pull_down_en = 1;                // enable pull-down mode
    io_conf.pull_up_en = 0;                  // disable pull-up mode
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;   // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;         // set as output mode
    io_conf.pin_bit_mask = GPIO_SWITCH_PIN_SEL; // bit mask of the pins that you want to set
    io_conf.pull_down_en = 1;                // enable pull-down mode
    io_conf.pull_up_en = 0;                  // disable pull-up mode
    gpio_config(&io_conf);

}