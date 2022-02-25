#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"

#define TIMER_DIVIDER         (16)  //硬件定时器分频倍数Hardware timer clock divider
// 定时器计数频率convert counter value to seconds
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)

typedef struct {
    int timer_group;//定时器组号
    int timer_idx;//定时器索引
    int alarm_interval;//定时器预警间隔
    bool auto_reload;//定时器是否自动重载
} example_timer_info_t;

/**
 * @brief A sample structure to pass events from the timer ISR to task
 *
 */
typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

static xQueueHandle s_timer_queue;//队列句柄

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 * inline：在定义函数的时候  将其定义为内联函数
 */
static void inline print_timer_counter(uint64_t counter_value)
{  
    printf("Counter: 0x%08x%08x\r\n", (uint32_t) (counter_value >> 32),
            (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    /* 
    Prepare basic event data that will be then sent back to task
    将本函数中由example_timer_info_t定义的结构体变量info（见第41行）传递给由
    example_timer_event_t定义的结构体变量evt（见第63行）
    注：建议将info变量更名为 info_temp 以便于区分
    如下:
    example_timer_info_t *info_temp = (example_timer_info_t *) args;
    example_timer_event_t evt = {
        .info.timer_group = info_temp->timer_group,
        .info.timer_idx = info_temp->timer_idx,
        .info.auto_reload = info_temp->auto_reload,
        .info.alarm_interval = info_temp->alarm_interval,
        .timer_counter_value = timer_counter_value
    };
     */
    example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    if (!info->auto_reload) {
        //如果不自动重载，则设置下次预警值为此时的预警值加上预警间隔
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        //更新预警值
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,//分频倍数
        .counter_dir = TIMER_COUNT_UP,//向上计数(1)/向下计数(0)
        .counter_en = TIMER_PAUSE,//定时器开(1)关(0)
        .alarm_en = TIMER_ALARM_EN,//定时器中断开(1)关(0)
        .auto_reload = auto_reload,//是否自动重载
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);//定时器计数起始值

    /*
    Configure the alarm value and the interrupt on alarm. 
    设置中断阈值及中断使能
    */
    timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(group, timer);//中断使能

    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    
    /*
    timer_isr_callback_add的参数中，timer_info会传递给
    timer_group_isr_callback的arg(详见51行)
    */
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);
}

void app_main(void)
{
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));

    //初始化两个定时器
    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 3);
    example_tg_timer_init(TIMER_GROUP_1, TIMER_0, false, 5);

    //测试前两个计数器是否准时
    example_tg_timer_init(TIMER_GROUP_1, TIMER_1, false, 1);


    while (1) {
        /*
        定义一个example_timer_event_t结构体变量evt来接收从回调函数中通过队列传回的数据
        (详见第84行：xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);)
        其中包括定时器组号：timer_group、定时器索引：timer_idx、
        定时器是否自动重载：auto_reload、定时器中断阈值：timer_interval_sec
        定时器当前计数值：timer_counter_value
        */
        example_timer_event_t evt;
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);

        printf("\n");
        printf("/*************************/\n");

        /* Print information that the timer reported an event */
        if (evt.info.auto_reload) {
            printf("Timer Group with auto reload\n");
        } else {
            printf("Timer Group without auto reload\n");
        }
        printf("Group[%d], timer[%d] alarm event\n", evt.info.timer_group, evt.info.timer_idx);

        /* Print the timer values passed by event */
        printf("------- EVENT TIME --------\n");
        print_timer_counter(evt.timer_counter_value);

        /* Print the timer values as visible by this task */
        printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        timer_get_counter_value(evt.info.timer_group, evt.info.timer_idx, &task_counter_value);
        print_timer_counter(task_counter_value);

        printf("当前消息队列个数：%d\n",uxQueueMessagesWaitingFromISR(s_timer_queue));
        printf("/*************************/\n");
        printf("\n");
    }
}


