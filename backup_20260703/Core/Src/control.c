/**
 * @file    control.c
 * @brief   BTS7960 H桥驱动控制
 *
 * BTS7960 控制逻辑:
 *   PA6 (TIM3_CH1) = IN1 → 加热片 (PWM 正向)
 *   PA7 (TIM3_CH2) = IN2 → TEC 制冷片 (PWM 反向)
 *   任一时刻只有 IN1 或 IN2 输出 PWM (软件互锁)
 *
 * 负载接法:
 *   电热片(+) ← OUT+ → TEC(+)
 *   电热片(-) ← GND  ← TEC(-)
 *   即 OUT+ 经加热片到 GND (加热), OUT- 经 TEC 到 GND (制冷)
 */

#include "control.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;

/* 启动两个 PWM 通道并初始化为 0 */
void Control_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* PA6 = IN1 = 加热 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* PA7 = IN2 = 制冷 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

/**
 * 根据 PID 输出值设置 PWM 占空比
 * PID 输出范围: [-100, +100] 对应 [-100%, +100%] PWM 占空比
 * 控制区段划分 (基于 PID 输出值):
 *   u > +100 → 全功率加热 (100%) — 仅防溢出
 *   +DEAD_ZONE < u ≤ +100 → PID 比例加热 (PWM = |u|%)
 *   -DEAD_ZONE ≤ u ≤ +DEAD_ZONE → 死区 (停止)
 *   -100 ≤ u < -DEAD_ZONE → PID 比例制冷 (PWM = |u|%)
 *   u < -100 → 全功率制冷 (100%) — 仅防溢出
 */
void Control_ProcessOutput(Control_HandleTypeDef *ctrl, float pid_out)
{
    float out = pid_out;

    /* 限幅到有效范围 */
    if (out > 100.0f) out = 100.0f;
    if (out < -100.0f) out = -100.0f;

    /* ── PID 比例加热 (输出正值) ── */
    if (out > DEAD_ZONE)
    {
        ctrl->mode = CONTROL_HEAT;
        ctrl->heat_duty = (uint8_t)(out);
        if (ctrl->heat_duty > 100) ctrl->heat_duty = 100;
        if (ctrl->heat_duty < 5) ctrl->heat_duty = 5;
        ctrl->cool_duty = 0;
    }
    /* ── 死区: PID 输出很小, 停止控温 ── */
    else if (out >= -DEAD_ZONE)
    {
        ctrl->mode = CONTROL_IDLE;
        ctrl->heat_duty = 0;
        ctrl->cool_duty = 0;
    }
    /* ── PID 比例制冷 (输出负值) ── */
    else
    {
        ctrl->mode = CONTROL_COOL;
        ctrl->cool_duty = (uint8_t)(-out);
        if (ctrl->cool_duty > 100) ctrl->cool_duty = 100;
        if (ctrl->cool_duty < 5) ctrl->cool_duty = 5;
        ctrl->heat_duty = 0;
    }

    Control_SetPower(ctrl->mode,
                     (ctrl->mode == CONTROL_HEAT) ? ctrl->heat_duty : ctrl->cool_duty);
}

/* 停止所有 PWM 输出并复位状态 */
void Control_Stop(Control_HandleTypeDef *ctrl)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    if (ctrl)
    {
        ctrl->mode = CONTROL_IDLE;
        ctrl->heat_duty = 0;
        ctrl->cool_duty = 0;
    }
}

/**
 * 设置 BTS7960 输出
 * @param mode  CONTROL_HEAT / CONTROL_COOL / CONTROL_IDLE
 * @param duty  占空比 0-100
 */
void Control_SetPower(int8_t mode, uint8_t duty)
{
    uint16_t pulse = (uint16_t)((uint32_t)duty * PWM_PERIOD / 100);
    if (pulse > PWM_PERIOD) pulse = PWM_PERIOD;

    if (mode == CONTROL_HEAT)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);  /* IN1 = PWM */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);      /* IN2 = LOW */
    }
    else if (mode == CONTROL_COOL)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);      /* IN1 = LOW */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);  /* IN2 = PWM */
    }
    else  /* IDLE */
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}
