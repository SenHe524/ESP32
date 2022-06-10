#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stdint.h"
#include "stdbool.h"
#include "math.h"  
#include "string.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sys/unistd.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "Register.h"

#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1 

void I2C_Init(uint8_t sclPin, uint8_t sdaPin) 
{
    i2c_config_t conf;                          //I2C 配置结构体
    conf.mode = I2C_MODE_MASTER;                //I2C 模式
    conf.sda_io_num = sdaPin;                   //SDA IO映射
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;    //SDA IO模式
    conf.scl_io_num = sclPin;                   //SCL IO映射
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;    //SCL IO模式
    conf.master.clk_speed = 400 * 1000;             //I2C CLK频率
    i2c_param_config(I2C_NUM_1, &conf);    //设置I2C1
    //注册I2C1服务即使能
    i2c_driver_install(I2C_NUM_1, conf.mode,0,0,0);
}
void Multiple_Read_Hardware(uint8_t SlaveAddress,  uint8_t Read_Address, uint8_t *rx_buf, size_t Len)
{
    // i2c_master_write_read_device(I2C_NUM_1, SlaveAddress, &Read_Address, 1, rx_buf, Len, 5 / portTICK_RATE_MS);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Read_Address, ACK_CHECK_EN);
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress + 1, ACK_CHECK_EN);
    //循环单字节读
    for(int i = 0; i < Len-1; i++)
    {
        i2c_master_read_byte(cmd, rx_buf++, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, rx_buf, I2C_MASTER_NACK);

    // //直接读Len个字节
    // i2c_master_read(cmd, rx_buf, Len, I2C_MASTER_LAST_NACK);

    //读Len-1个字节 然后读1个字节
    // i2c_master_read(cmd, rx_buf, Len - 1, I2C_MASTER_ACK);
    // i2c_master_read(cmd, rx_buf, 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		ESP_LOGE("WRITE_ERR", "IIC Read Bytes failed:%s,SlaveAddress = %x\n", esp_err_to_name(ret), SlaveAddress);
	}

}

void Single_Write_hardware(uint8_t SlaveAddress, uint8_t Write_Address, uint8_t Data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Write_Address, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		ESP_LOGE("WRITE_ERR", "IIC Read Bytes failed:%s,SlaveAddress = %x\n", esp_err_to_name(ret), SlaveAddress);
	}

}

uint8_t Single_Read_hardware(uint8_t SlaveAddress, uint8_t Read_Address)		
{
	uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress, 1);
	i2c_master_write_byte(cmd, Read_Address, 1);
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress + 1, 1);
    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		ESP_LOGE("WRITE_ERR", "IIC Read Bytes failed:%s,SlaveAddress = %x\n", esp_err_to_name(ret), SlaveAddress);
	}
	return data;
}  


