/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF  1100  //参考电压  
#define NO_OF_SAMPLES  128   //Multisampling
//定义一个ADC的参数结构体
static esp_adc_cal_characteristics_t *adc_chars;

#if CONFIG_IDF_TARGET_ESP32
//选择采样通道：ADC_6
static const adc_channel_t channel = ADC_CHANNEL_6; 
//选择采样精度：12位   
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;  
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
//选择ADC的通道的衰减模式，此参数影响测量电压的范围
//ADC_ATTEN_DB_(0,2_5,6,11)分别对应了 0dB 2.5dB 6dB 11dB衰减
static const adc_atten_t atten = ADC_ATTEN_DB_11;
//选择ADC_1
static const adc_unit_t unit = ADC_UNIT_1;

//检查eFuse是否准备就绪
static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse 
    //检查两点值(Two Point)是否烧录进 eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    //检查参考电压模式是否烧录进 eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

//打印使用的特征值类别信息，此部分函数为 使用ADC时 非必须代码
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


void app_main(void)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        //配置ADC_1的转换精度
        adc1_config_width(width);
        //配置ADC_1的通道衰减模式
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC

    /*  在内存中分配 1 个长度为 sizeof(esp_adc_cal_characteristics_t) 
        的连续空间，并将起始地址指针返回给adc_chars
        */
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //设置ADC的通道、衰减模式、精度、在eFuse值不可用时的参考电压、存储ADC参数特性的结构体指针
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        //定义一个接收获取到的ADC数字量值的变量
        uint32_t adc_reading = 0;

        //Multisampling
        //采样NO_OF_SAMPLES(128)次，然后取平均值
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        //将ADC_1的取平均读数：adc_reading 转换为以mV为单位的电压
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));//
    }
}
