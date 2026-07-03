/**
 * @file    pid.c
 * @brief   位置式 PID 控制器
 *
 * 算法:  u(k) = Kp*e(k) + Ki*Σ[e(j)*T] - Kd*[y(k)-y(k-1)]/T
 *
 * 改进策略:
 *   1. 积分分离: |e|>3°C 时 Ki=0, 防止大幅超调
 *   2. 微分对测量值: 避免设定值变化时的微分冲击
 *   3. 独立积分限幅: Ki * integral_limit = 80% output_limit
 *   4. 抗积分饱和: 输出限幅时停止积分累积
 *   5. 输出限幅: [-100, 100], 对应 PWM 0%~100%
 */

#include "pid.h"

/* 初始化 PID 参数并清零内部状态 */
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd,
              float T, float lim_max, float lim_min)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T_sample = T;
    pid->limit_max = lim_max;
    pid->limit_min = lim_min;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->out = 0.0f;
}

/* 复位 PID: 清零积分、历史测量值、输出 */
void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->out = 0.0f;
}

/**
 * 执行一次 PID 计算 (位置式, 微分对测量值)
 * @param setpoint    目标值 (°C)
 * @param measurement 当前测量值 (°C)
 * @return 控制量 [-100, +100], 正=加热, 负=制冷
 */
float PID_Calculate(PID_HandleTypeDef *pid, float setpoint, float measurement)
{
    if (pid->T_sample <= 0.0f) return 0.0f;

    float error = setpoint - measurement;
    float p_term, i_term, d_term;
    float integral_sep_threshold = 3.0f;

    /* 独立积分限幅: 确保 Ki * integral_limit ≤ 80% of output_limit */
    float integral_limit = pid->limit_max * 0.8f / pid->Ki;
    if (integral_limit < 0) integral_limit = -integral_limit;

    /* ── 积分分离 ── */
    if (error > integral_sep_threshold || error < -integral_sep_threshold)
    {
        i_term = 0.0f;
    }
    else
    {
        pid->integral += error * pid->T_sample;
        if (pid->integral > integral_limit) pid->integral = integral_limit;
        if (pid->integral < -integral_limit) pid->integral = -integral_limit;
        i_term = pid->Ki * pid->integral;
    }

    /* ── P 项: 比例 ── */
    p_term = pid->Kp * error;

    /* ── D 项: 微分对测量值 (避免设定值突变时的微分冲击) ── */
    d_term = -pid->Kd * (measurement - pid->prev_measurement) / pid->T_sample;
    pid->prev_measurement = measurement;

    pid->out = p_term + i_term + d_term;

    /* ── 抗积分饱和: 输出限幅时反方向修正积分 ── */
    if (pid->out > pid->limit_max)
    {
        pid->out = pid->limit_max;
        if (error > 0) pid->integral -= error * pid->T_sample;
    }
    if (pid->out < pid->limit_min)
    {
        pid->out = pid->limit_min;
        if (error < 0) pid->integral -= error * pid->T_sample;
    }

    return pid->out;
}
