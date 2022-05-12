#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"

#include "DHT11.h"


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

uint8_t OneWire_Init(void)		//单总线初始化
{
	uint8_t AckBit;

	gpio_set_direction(OneWire_DQ, GPIO_MODE_OUTPUT);
    gpio_set_level(OneWire_DQ, 1);//初始化电平
	ets_delay_us(5);
	gpio_set_level(OneWire_DQ, 0);//拉低电平

	ets_delay_us(20*1000);//延时20ms

	gpio_set_level(OneWire_DQ, 1);//拉高电平

	ets_delay_us(25);//延时30us

	gpio_set_direction(OneWire_DQ, GPIO_MODE_INPUT);//设置IO口为输入

	AckBit = gpio_get_level(OneWire_DQ);//读取IO口电平
	//延时80us，等待响应信号的低电平结束
	//此响应信号为83us的低电平，紧跟87us的高电平通知MCU准备接收数据
	ets_delay_us(80);
	gpio_set_level(OneWire_DQ, 1); //拉高电平
	return AckBit;
}


uint8_t OneWire_ReceiveByte(void)			 //接收一个字节
{
	uint8_t i;
	uint8_t Byte = 0x00;
	//循环8此读取一字节的每一位
	for(i = 0; i < 8; i++)
	{
		//等待数据帧中的低电平结束
		while(!gpio_get_level(OneWire_DQ)){//循环读取IO口电平，直到读取电平为1
			ets_delay_us(3);
		}

		//数据帧0/1区别为54us后的高电平时间
		//0：高电平时间为23-27us
		//1：低电平时间为68-74us
		//延时35us后读取电平，此时若电平为一，则数据帧为1，反之则为0
		ets_delay_us(35);
		if(gpio_get_level(OneWire_DQ)){Byte|=(0x80 >> i);}
		//若数据帧为1，则延时等待高电平过去，数据帧为0时，跳过
		while(gpio_get_level(OneWire_DQ)){
			ets_delay_us(3);
		}
	}
	return Byte;
}


void data_read(void)
{
	uint8_t data_temp[5] = {0};
	uint8_t temp = 0;
	if(!OneWire_Init())
	{
		gpio_set_direction(OneWire_DQ, GPIO_MODE_INPUT);
		//延时等待DHT11的高电平结束
		while(gpio_get_level(OneWire_DQ)){//循环读取IO口电平，直到读取电平为0
			ets_delay_us(3);
		}
		//循环读取温湿度数据
		for(int i = 0; i < 5; i++)
		{
			data_temp[i] = OneWire_ReceiveByte();
		}
		//读取结束后，设置IO口为输出高电平
		gpio_set_direction(OneWire_DQ, GPIO_MODE_OUTPUT);
		gpio_set_level(OneWire_DQ, 1);
		//检验校验和
		temp = data_temp[0] + data_temp[1] + data_temp[2] + data_temp[3];
		if(data_temp[4] == temp)
		{//校验成功，则打印温湿度
			printf("湿度为：%d.%d\t\t",data_temp[0], data_temp[1]);
			printf("温度为：%d.%d\n",data_temp[2], data_temp[3]);
		}
		else
		{//校验失败，则提示读取错误
			printf("Read Err!\n");
		}
	}
	else
	{
		printf("初始化失败！\n");
	}
	
}
