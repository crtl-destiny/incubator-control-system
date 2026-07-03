#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void Error_Handler(void);

/* ========== 硬件引脚映射 ========== */
#define BUZZER_PIN      GPIO_PIN_8    /* 蜂鸣器 PA8, 低电平有效 */
#define BUZZER_PORT     GPIOA

/* ========== 温度控制参数 ========== */
#define TEMP_MIN        5.0f          /* 最低设定温度 */
#define TEMP_MAX        50.0f         /* 最高设定温度 */
#define TEMP_DEFAULT    37.0f         /* 默认目标温度 (人体体温培养) */

/* ========== 报警阈值 ========== */
#define ALARM_TEMP_HIGH 2.0f          /* 温度偏高阈值 */
#define ALARM_TEMP_LOW  2.0f          /* 温度偏低阈值 */

/* ========== 时序参数 ========== */
#define CONTROL_CYCLE_MS    200       /* 控制周期 200ms */
#define USB_REPORT_CYCLE    1000      /* USB上报周期 1s */

/* ========== 系统运行状态 ========== */
typedef enum
{
    SYSTEM_STATE_IDLE,      /* 空闲 - 不控温 */
    SYSTEM_STATE_RUNNING,   /* 运行 - PID自动控温 */
    SYSTEM_STATE_SETTING,   /* 设置 - 按键输入温度值 */
    SYSTEM_STATE_ALARM      /* 报警 - 温度超限 */
} SystemState;

/* 全局变量 (在 main.c 中定义) */
extern volatile SystemState sys_state;
extern volatile float current_temp;   /* 当前实测温度 */
extern volatile float target_temp;    /* 目标设定温度 */
extern volatile uint32_t run_time_sec; /* 已运行秒数 */

#ifdef __cplusplus
}
#endif

#endif
