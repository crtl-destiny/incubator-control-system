/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* ========== 硬件引脚映射 ========== */
#define BUZZER_PIN      GPIO_PIN_8    /* 蜂鸣器 PA8, 经 NPN 三极管驱动 5V 有源蜂鸣器 */
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
#define USB_REPORT_CYCLE    1000      /* 串口上报周期 1s */

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
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
