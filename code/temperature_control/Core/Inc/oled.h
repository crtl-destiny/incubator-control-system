#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

/* SSD1306 OLED I2C 参数 */
#define OLED_ADDR       0x3C   /* I2C 设备地址 (7位) */
#define OLED_I2C        hi2c1  /* 使用 I2C1 外设 */
#define SSD1306_CMD     0x00   /* 命令模式 */
#define SSD1306_DATA    0x40   /* 数据模式 */

/* 屏幕分辨率 */
#define OLED_WIDTH      128    /* 列数 */
#define OLED_HEIGHT     64    /* 行数 */

/* 初始化 */
void OLED_Init(void);
void OLED_SendCmd(uint8_t cmd);
void OLED_SendData(uint8_t data);

/* 光标与刷新 */
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_Clear(void);
void OLED_Display(void);

/* 字符串/数值显示 (size=6:小字体, size=8:大字体) */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const char* str, uint8_t size);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t intLen, uint8_t decLen, uint8_t size);
void OLED_Printf(uint8_t x, uint8_t y, const char* fmt, ...);

#endif
