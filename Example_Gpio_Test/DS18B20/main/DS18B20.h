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

void temperature_convert(void);

float temperature_read(uint8_t addr);

#endif