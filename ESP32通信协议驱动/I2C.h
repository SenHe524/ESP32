#ifndef _I2C_H_
#define _I2C_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"


void gpio_init(void);

int Single_Write(uint8_t SlaveAddress, uint8_t Write_Address, uint8_t Data);		     

uint8_t Single_Read(uint8_t SlaveAddress, uint8_t Read_Address);


#endif