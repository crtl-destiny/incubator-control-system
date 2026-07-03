#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

/* BTS7960 PWM 参数 */
#define PWM_PERIOD  7200        /* TIM3 ARR, 72MHz/10kHz=7200 */
#define PWM_FREQ    10000       /* PWM 频率 10kHz */

/* 死区阈值 (%): PID 输出绝对值 ≤ DEAD_ZONE 时停止加热/制冷 */
#define DEAD_ZONE   5

/* 控制模式 */
#define CONTROL_HEAT   1
#define CONTROL_COOL   -1
#define CONTROL_IDLE   0

/* 控制输出句柄 */
typedef struct
{
    int8_t  mode;       /* 当前模式: HEAT/COOL/IDLE */
    uint8_t heat_duty;  /* 加热PWM占空比 0-100% */
    uint8_t cool_duty;  /* 制冷PWM占空比 0-100% */
} Control_HandleTypeDef;

void Control_Init(void);                                    /* 初始化PWM输出 */
void Control_ProcessOutput(Control_HandleTypeDef *ctrl, float pid_out);  /* 根据PID输出设PWM */
void Control_Stop(Control_HandleTypeDef *ctrl);             /* 停止所有输出并复位状态 */
void Control_SetPower(int8_t mode, uint8_t duty);           /* 直接设置PWM (mode: HEAT/COOL/IDLE) */

#endif
