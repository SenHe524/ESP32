#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifndef _ONEWIRE_H
#define _ONEWIRE_H

void gpio_init(void);

uint8_t OneWire_Init();		//单总线初始化

void OneWire_SendByte(uint8_t Byte);  //发送一个字节

uint8_t OneWire_ReceiveByte();			 //接收一个字节



#endif