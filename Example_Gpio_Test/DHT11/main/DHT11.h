#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifndef _DHT11_H
#define _DHT11_H

void gpio_init(void);

void data_read(void);

#endif