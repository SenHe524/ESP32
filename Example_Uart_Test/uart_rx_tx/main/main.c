
//串口接收
    /*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

#define UART_NUM_rx UART_NUM_0
#define rx_TXD_PIN (GPIO_NUM_1)
#define rx_RXD_PIN (GPIO_NUM_3)

static void rx_task(void *arg);
void uart0_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_rx, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_rx, &uart_config);
    uart_set_pin(UART_NUM_rx, rx_TXD_PIN, rx_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}


static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        size_t buf_ok;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, &buf_ok));
        if(buf_ok > 0)
        {
            ESP_LOGI(RX_TASK_TAG, "read Begin");
            const int rxBytes = uart_read_bytes(UART_NUM_rx, data, buf_ok, 1000 / portTICK_RATE_MS);
            if (rxBytes > 0) {
                data[rxBytes] = 0;
                ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
    }
    free(data);
}

void app_main(void)
{
    uart0_init();//com7
}
    */

//串口队列接收
    /*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

static QueueHandle_t uart_queue; //串口队列句柄

#define UART_NUM_rx UART_NUM_0
#define rx_TXD_PIN (GPIO_NUM_1)
#define rx_RXD_PIN (GPIO_NUM_3)

static void rx_task(void *arg);
void uart0_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_rx, RX_BUF_SIZE * 2, 0, 20, &uart_queue, 0);
    uart_param_config(UART_NUM_rx, &uart_config);
    uart_set_pin(UART_NUM_rx, rx_TXD_PIN, rx_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}


static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    uart_event_t event;

    for(;;)
    {
        if(xQueueReceive(uart_queue, (void *)&event, ((portTickType)portMAX_DELAY)))
        {
            switch(event.type) {//各种串口事件
                case UART_DATA: {
                    size_t buf_ok;
                    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_rx, &buf_ok));
                    if(buf_ok > 0)
                    {
                        ESP_LOGI(RX_TASK_TAG, "read Begin");
                        const int rxBytes = uart_read_bytes(UART_NUM_rx, data, buf_ok, 1000 / portTICK_RATE_MS);
                        if (rxBytes > 0) {
                            data[rxBytes] = 0;
                            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
                            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                        }
                    }
                    break;
                }
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF: //硬件fifo溢出
                    ESP_LOGI(RX_TASK_TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_rx);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL: //环形缓冲区满
                    ESP_LOGI(RX_TASK_TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_rx);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(RX_TASK_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(RX_TASK_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(RX_TASK_TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    break;
                //Others
                default:
                    ESP_LOGI(RX_TASK_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}

void app_main(void)
{
    uart0_init();//com8
}
    */

//串口发送
    /*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

#define UART_NUM_tx UART_NUM_2
#define tx_TXD_PIN (GPIO_NUM_17)
#define tx_RXD_PIN (GPIO_NUM_16)

static void tx_task(void *arg);

void uart2_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_tx, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_tx, &uart_config);
    uart_set_pin(UART_NUM_tx, tx_TXD_PIN, tx_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_tx, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    uart2_init();//com5
}
    */