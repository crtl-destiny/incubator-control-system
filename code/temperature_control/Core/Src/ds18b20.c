/**
 * @file    ds18b20.c
 * @brief   DS18B20 数字温度传感器驱动 (1-Wire 单总线协议)
 *
 * 支持两种工作模式:
 *   阻塞模式: DS18B20_GetTemperature() — 内部等待 750ms
 *   非阻塞模式: DS18B20_StartConversion() + DS18B20_ReadResult()
 *              — 适合嵌入 200ms 控制周期
 */

#include "ds18b20.h"

/* DWT 循环计数器 (42位, 72MHz 下每个 tick = 1/72 us) */
static uint32_t DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return DWT->CYCCNT;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* 将数据线拉低 (配置为推挽输出并输出低电平) */
void DS18B20_SetLow(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = DS18B20_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DS18B20_PORT, &gpio);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
}

/* 将数据线拉高 (配置为推挽输出并输出高电平) */
void DS18B20_SetHigh(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = DS18B20_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DS18B20_PORT, &gpio);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
}

/* 读取数据线电平 (配置为浮空上拉输入) */
uint8_t DS18B20_ReadPin(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = DS18B20_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DS18B20_PORT, &gpio);
    return HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN);
}

/* 复位脉冲并检测存在脉冲: 返回 1=设备存在, 0=无设备 */
int DS18B20_ResetPulse(void)
{
    int presence;
    DS18B20_SetLow();
    delay_us(480);      /* 主机拉低 480us 产生复位脉冲 */
    DS18B20_SetHigh();
    delay_us(70);       /* 释放总线, 等待设备应答 */
    presence = (DS18B20_ReadPin() == GPIO_PIN_RESET) ? 1 : 0;
    delay_us(410);
    return presence;
}

/* 向 DS18B20 写入一个字节 (LSB first) */
void DS18B20_WriteByte(uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        if (byte & (1 << i))
        {
            DS18B20_SetLow();
            delay_us(1);
            DS18B20_SetHigh();
            delay_us(60);
        }
        else
        {
            DS18B20_SetLow();
            delay_us(60);
            DS18B20_SetHigh();
            delay_us(2);
        }
    }
}

/* 从 DS18B20 读取一个字节 (LSB first) */
uint8_t DS18B20_ReadByte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++)
    {
        DS18B20_SetLow();
        delay_us(2);
        DS18B20_SetHigh();
        delay_us(2);
        if (DS18B20_ReadPin())
        {
            byte |= (1 << i);
        }
        delay_us(60);
    }
    return byte;
}

/* 初始化 DS18B20: 启用 DWT 并检测设备存在 */
int DS18B20_Init(void)
{
    DWT_Init();
    DS18B20_SetHigh();
    HAL_Delay(1);
    return DS18B20_ResetPulse();
}

/* 非阻塞: 仅启动温度转换 (不等待) */
void DS18B20_StartConversion(void)
{
    DS18B20_ResetPulse();
    DS18B20_WriteByte(DS18B20_SKIP_ROM);
    DS18B20_WriteByte(DS18B20_CONVERT_T);
}

/* 非阻塞: 读取转换结果 (需确保转换已完成) */
float DS18B20_ReadResult(void)
{
    uint8_t tempL, tempH;
    int16_t raw;

    DS18B20_ResetPulse();
    DS18B20_WriteByte(DS18B20_SKIP_ROM);
    DS18B20_WriteByte(DS18B20_READ_SCRATCHPAD);
    tempL = DS18B20_ReadByte();
    tempH = DS18B20_ReadByte();

    raw = (int16_t)((tempH << 8) | tempL);
    return raw * 0.0625f;
}

/**
 * 阻塞式读取温度 (兼容旧接口, 750ms 等待)
 * 新代码建议使用非阻塞模式:
 *   DS18B20_StartConversion();
 *   HAL_Delay(750);
 *   temp = DS18B20_ReadResult();
 */
float DS18B20_GetTemperature(void)
{
    DS18B20_StartConversion();
    HAL_Delay(750);
    return DS18B20_ReadResult();
}
