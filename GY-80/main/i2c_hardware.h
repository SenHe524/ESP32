#ifndef _H_I2C_H_
#define _H_I2C_H_
#include "stdint.h"
#include "stdbool.h"



void I2C_Init(uint8_t sclPin, uint8_t sdaPin); 

void Multiple_Read_Hardware(uint8_t SlaveAddress,  uint8_t Read_Address, uint8_t *rx_buf, size_t Len);
uint8_t Single_Read_hardware(uint8_t SlaveAddress, uint8_t Read_Address);
void Single_Write_hardware(uint8_t SlaveAddress, uint8_t Write_Address, uint8_t Data);

#endif
