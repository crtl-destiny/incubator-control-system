#ifndef __KEY_H__
#define __KEY_H__

#include "main.h"

/* 4×3 矩阵键盘尺寸 */
#define KEY_ROWS    4
#define KEY_COLS    3

/* 无按键按下返回值 */
#define KEY_NONE    0xFF

/* 特殊按键编码: * = 0x0A(设置/退格), # = 0x0B(确认/启停) */
typedef enum
{
    KEY_MODE_SET   = 0x0A,
    KEY_MODE_ENTER = 0x0B
} KeySpecial;

void Key_Init(void);
uint8_t Key_Scan(void);      /* 扫描一次键盘，返回按键值 */
uint8_t Key_GetValue(void);  /* 等同 Key_Scan */

#endif
