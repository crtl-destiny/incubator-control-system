#ifndef __PID_H__
#define __PID_H__

#include "main.h"

/* PID 控制器句柄 */
typedef struct
{
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */
    float T_sample;     /* 采样周期 (s) */
    float limit_max;    /* 输出上限 */
    float limit_min;    /* 输出下限 */
    float integral;     /* 积分累积值 */
    float prev_measurement; /* 上一次测量值 (用于微分项) */
    float out;          /* 当前输出值 */
} PID_HandleTypeDef;

/* 初始化PID (含参数、采样周期、限幅) */
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float T, float lim_max, float lim_min);
/* 复位PID (清零积分、历史误差) */
void PID_Reset(PID_HandleTypeDef *pid);
/* 执行一次PID计算: 输入设定值和测量值, 返回控制量 */
float PID_Calculate(PID_HandleTypeDef *pid, float setpoint, float measurement);

#endif
