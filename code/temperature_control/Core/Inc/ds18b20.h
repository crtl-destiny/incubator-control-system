#ifndef __DS18B20_H__
#define __DS18B20_H__

#include "main.h"

/* DS18B20 数据引脚: PA0 */
#define DS18B20_PIN      GPIO_PIN_0
#define DS18B20_PORT     GPIOA

/* DS18B20 常用命令 */
#define DS18B20_SKIP_ROM         0xCC   /* 跳过ROM匹配（单设备时使用） */
#define DS18B20_CONVERT_T        0x44   /* 启动温度转换 */
#define DS18B20_READ_SCRATCHPAD  0xBE   /* 读取暂存器（含温度数据） */

/* 底层引脚操作 */
void     DS18B20_SetLow(void);
void     DS18B20_SetHigh(void);
uint8_t  DS18B20_ReadPin(void);

/* 1-Wire 总线时序 */
void     DS18B20_Reset(void);
int      DS18B20_ResetPulse(void);
void     DS18B20_WriteByte(uint8_t byte);
uint8_t  DS18B20_ReadByte(void);

/* 高层驱动接口 */
int      DS18B20_Init(void);

/* 阻塞模式 (内部等待 750ms) */
float    DS18B20_GetTemperature(void);

/* 非阻塞模式 (适合 200ms 控制周期) */
void     DS18B20_StartConversion(void);   /* 启动转换, 立即返回 */
float    DS18B20_ReadResult(void);        /* 读取结果 (需确保已等待 750ms) */

#endif
