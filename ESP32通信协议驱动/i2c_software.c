#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sys/unistd.h"

#include "i2c_software.h"

#define I2C_SCL 18
#define I2C_SDA 19
#define PIN_SELECT_I2C (1ULL << I2C_SCL) | (1ULL << I2C_SDA)

/**
 * @brief   I2C引脚初始化
 * @param   无
 * @return  
 *          无
 */
void gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // interrupt of rising edge
    io_conf.pin_bit_mask = PIN_SELECT_I2C; // bit mask of the pins, use GPIO4/5 here
    io_conf.mode = GPIO_MODE_INPUT;        // set as input mode
    io_conf.pull_up_en = 1;                // enable pull-up mode
    io_conf.pull_down_en = 0;              // disable pull-down mode
    gpio_config(&io_conf);
}

/**
 * @brief   I2C起始信号
 * @param   无
 * @return  
 *          无
 */
void I2C_Start(void)
{
    //电平拉高初始化
    gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_SDA, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_SCL, 1);
    gpio_set_level(I2C_SDA, 1);
    ets_delay_us(2);
    //拉低SCL、SDA的电平 模拟开始信号
    gpio_set_level(I2C_SDA, 0);
    ets_delay_us(1);
    gpio_set_level(I2C_SCL, 0);
}

/**
 * @brief   I2C停止信号
 * @param   无
 * @return  
 *          无
 */
void I2C_Stop(void)
{
    gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_SDA, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_SCL, 0);
    gpio_set_level(I2C_SDA, 0);
    ets_delay_us(1);
    gpio_set_level(I2C_SCL, 1);
    gpio_set_level(I2C_SDA, 1);
    gpio_set_direction(I2C_SCL, GPIO_MODE_INPUT);
    gpio_set_direction(I2C_SDA, GPIO_MODE_INPUT);
}

/**
 * @brief   I2C发送单字节
 * @param   Byte 要发送的字节
 * @return  
 *          无
 */
void I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_SDA, GPIO_MODE_OUTPUT);
    for (i = 0; i < 8; i++)
    {
        gpio_set_level(I2C_SDA, Byte & (0x80 >> i));
        gpio_set_level(I2C_SCL, 1);
        gpio_set_level(I2C_SCL, 0);
    }
}

/**
 * @brief   I2C接收单字节
 * @param   无
 * @return  
 *          Byte：接收到的字节
 */
uint8_t I2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;
    gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_SDA, GPIO_MODE_INPUT);
    gpio_set_level(I2C_SDA, 1); //置1，释放总线
    for (i = 0; i < 8; i++)
    {
        gpio_set_level(I2C_SCL, 1); //置1，高电平读取SDA
        if (gpio_get_level(I2C_SDA)){
            Byte |= (0x80 >> i);
            }
        gpio_set_level(I2C_SCL, 0);
    }
    return Byte;
}

/**
 * @brief   I2C发送应答
 * @param   AckBit 要发送的应答
 * @return  
 *          无
 */
void I2C_SendAck(uint8_t AckBit)
{
    gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_SDA, GPIO_MODE_OUTPUT);
    //将AckBit放置在SDA上，等待从机读取
    gpio_set_level(I2C_SDA, AckBit);
    gpio_set_level(I2C_SCL, 1); // SCL置1，等待从机读取
    gpio_set_level(I2C_SCL, 0);
}

/**
 * @brief   I2C接收应答
 * @param   无
 * @return  
 *          AckBit：接收到的应答
 */
uint8_t I2C_ReceiveAck(void)
{
    uint8_t AckBit = 0;
    gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_SDA, GPIO_MODE_INPUT);

    // SDA置1，释放SDA，此后SDA变化由从机决定
    gpio_set_level(I2C_SDA, 1);
    // SCL置1，在SCL高电平期间读取SDA
    gpio_set_level(I2C_SCL, 1);
    if (gpio_get_level(I2C_SDA))
    {
        AckBit = 1;
    }
    gpio_set_level(I2C_SCL, 0);
    return AckBit;
}

/**
 * @brief   I2C单字节写
 * @param   SlaveAddress 从机地址
 * @param   Write_Address 要写入的地址
 * @param   Data 要写入的数据
 * @return  
 *          0：写入失败
 *          1：写入成功
 */
int Single_Write(uint8_t SlaveAddress, uint8_t Write_Address, uint8_t Data)
{
    I2C_Start();
    //发送设备地址+写信号
    I2C_SendByte(SlaveAddress);
    if (I2C_ReceiveAck())
    {
        I2C_Stop();
        return 0;
    }
    I2C_SendByte(Write_Address); //从机写入地址
    I2C_ReceiveAck();
    I2C_SendByte(Data);
    I2C_ReceiveAck();
    I2C_Stop();
    ets_delay_us(5 * 1000); //延时5ms
    return 1;
}

/**
 * @brief   I2C单字节读
 * @param   SlaveAddress 从机地址
 * @param   Write_Address 要读取的地址
 * @return  
 *          Data：读取到的数据
 */
uint8_t Single_Read(uint8_t SlaveAddress, uint8_t Read_Address)
{
    uint8_t Data;
    I2C_Start();
    I2C_SendByte(SlaveAddress); //发送从机写地址
    if (I2C_ReceiveAck())
    {
        I2C_Stop();
        printf("I'm Here!--------No Ack!\n");
        return 0;
    }
    I2C_SendByte(Read_Address); //要读取的地址
    I2C_ReceiveAck();

    I2C_Start();
    I2C_SendByte(SlaveAddress + 1); //发送从机读地址
    I2C_ReceiveAck();
    Data = I2C_ReceiveByte();
    I2C_SendAck(1);
    I2C_Stop();
    return Data;
}
int Multiple_Read(uint8_t SlaveAddress, uint8_t Read_Address, uint8_t *Data_temp, uint8_t Len)
{
    I2C_Start();
    I2C_SendByte(SlaveAddress); //发送从机写地址
    if (I2C_ReceiveAck())
    {
        I2C_Stop();
        printf("I'm Here!--------No Ack!\n");
        return 0;
    }
    I2C_SendByte(Read_Address); //要读取的地址
    I2C_ReceiveAck();

    I2C_Start();
    I2C_SendByte(SlaveAddress + 1); //发送从机读地址
    I2C_ReceiveAck();
    for(int i = 0; i < Len - 1; i++)
    {

        Data_temp[i] = I2C_ReceiveByte();
        I2C_SendAck(0);
    }
    Data_temp[Len - 1] = I2C_ReceiveByte();
    I2C_SendAck(1);

    I2C_Stop();
    return 1;
}
