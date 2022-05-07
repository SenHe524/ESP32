#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifndef _DS18B20_H
#define _DB18B20_H

void gpio_init(void);

uint8_t OneWire_Init();		//单总线初始化

void OneWire_SendByte(uint8_t Byte);	  //发送一个字节

uint8_t OneWire_ReceiveByte();			 //接收一个字节

#endif