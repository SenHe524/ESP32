#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"

#include "DS18B20.h"


#define OneWire_DQ    4
#define GPIO_PIN_SEL (1ULL<<OneWire_DQ)
#define DS18B20_SKIP_ROM         0xCC
#define DS18B20_CONVERT_T        0x44
#define DS18B20_READ_SCRATCHPAD  0xBE


void gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;     // interrupt of rising edge
    io_conf.pin_bit_mask = GPIO_PIN_SEL; // bit mask of the pins, use GPIO4/5 here
    io_conf.mode = GPIO_MODE_INPUT;            // set as input mode
    io_conf.pull_up_en = 1;                    // enable pull-up mode
    io_conf.pull_down_en = 0;                  // disable pull-down mode
    gpio_config(&io_conf);
}

uint8_t OneWire_Init()		//单总线初始化
{
	uint8_t AckBit;

	gpio_set_direction(OneWire_DQ, GPIO_MODE_OUTPUT);
    gpio_set_level(OneWire_DQ, 1);//初始化电平
	ets_delay_us(5);
	gpio_set_level(OneWire_DQ, 0);//拉低电平

	ets_delay_us(500);//延时500us

	gpio_set_level(OneWire_DQ, 1);//拉高电平

	ets_delay_us(70);//延时70us

	gpio_set_direction(OneWire_DQ, GPIO_MODE_INPUT);

	AckBit = gpio_get_level(OneWire_DQ);

	ets_delay_us(500);//延时500us
	gpio_set_level(OneWire_DQ, 1); 
	return AckBit;
}
void OneWire_SendBit(uint8_t Bit)	   //发送一位
{
    
    gpio_set_direction(OneWire_DQ, GPIO_MODE_OUTPUT);
	//开启时序
	gpio_set_level(OneWire_DQ, 0);//拉低电平

	ets_delay_us(8);
	gpio_set_level(OneWire_DQ, Bit);
	ets_delay_us(60);
	gpio_set_level(OneWire_DQ, 1);
	ets_delay_us(8);
	gpio_set_direction(OneWire_DQ, GPIO_MODE_INPUT);

}
uint8_t OneWire_ReceiveBit()		//接收一位
{
	uint8_t Bit;

	gpio_set_direction(OneWire_DQ, GPIO_MODE_OUTPUT);
	//开启时序
	gpio_set_level(OneWire_DQ, 0);//拉低电平

	ets_delay_us(5); //延时5us
	gpio_set_direction(OneWire_DQ, GPIO_MODE_INPUT);
	gpio_set_level(OneWire_DQ, 1); //释放电平，此后电平变化由从机决定，
	ets_delay_us(10);//延时10us，进行采样，将采样结果赋给Bit
	Bit = gpio_get_level(OneWire_DQ);

	ets_delay_us(50);//延时50us
	gpio_set_level(OneWire_DQ, 1);
	return Bit;

}

void OneWire_SendByte(uint8_t Byte)	  //发送一个字节
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		OneWire_SendBit(Byte&(0x01<<i));//Byte&(0x01<<i)将Byte的第i+1位发送出去（低位在前先发送）
	}
}
uint8_t OneWire_ReceiveByte()			 //接收一个字节
{
	uint8_t i;
	uint8_t Byte = 0x00;
	for(i=0;i<8;i++)
	{
		if(OneWire_ReceiveBit()){Byte|=(0x01<<i);};
	}
	return Byte;
}

void temperature_convert(void)
{
	while(OneWire_Init()) {
        printf("正在初始化...\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
	OneWire_SendByte(DS18B20_SKIP_ROM);//跳过ROM
	OneWire_SendByte(DS18B20_CONVERT_T);//让DS18B20将温度转换为我们所需要的数值

	ets_delay_us(750 * 1000);//延时750ms，以待DS18B20将温度转换完成

}

float temperature_read(uint8_t addr)
{
	uint8_t T_LOW, T_HIGH;
	float T;

	while(OneWire_Init()) {
		printf("正在初始化...\n");
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
	OneWire_SendByte(DS18B20_SKIP_ROM);//跳过ROM
	OneWire_SendByte(addr);//读取温度寄存器
	T_LOW = OneWire_ReceiveByte();
	T_HIGH = OneWire_ReceiveByte();

	T = ((T_HIGH << 8) | T_LOW)/16.0;

	return T;
}
