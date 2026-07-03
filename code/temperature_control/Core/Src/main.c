/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 生化培养箱控制系统主程序
  *
  * 系统功能:
  *   1. DS18B20 温度传感器实时测温
  *   2. 4×3 矩阵键盘设定目标温度
  *   3. 位置式 PID 算法计算控制量
  *   4. BTS7960 H桥驱动加热片/TEC制冷片
  *   5. OLED 显示温度、状态、运行时间
 *   6. USART1 串口与上位机通信
 *   7. 蜂鸣器超限报警
 *
 * 主循环周期: 200ms, 数据上报周期: 1s
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "oled.h"
#include "key.h"
#include "pid.h"
#include "control.h"
#include "usart_com.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ========== 全局变量 (被 main.h 声明为 extern) ========== */
volatile SystemState sys_state = SYSTEM_STATE_IDLE;  /* 系统运行状态 */
volatile float current_temp = 0.0f;                  /* 当前实测温度 (°C) */
volatile float target_temp = TEMP_DEFAULT;           /* 目标设定温度 (°C) */
volatile uint32_t run_time_sec = 0;                  /* 累计运行秒数 */

/* ========== 模块句柄 ========== */
static PID_HandleTypeDef hpid;          /* PID 控制器 */
static Control_HandleTypeDef hctrl;     /* BTS7960 控制输出 */

/* ========== 内部状态变量 ========== */
static float pid_out = 0.0f;            /* PID 计算输出值 */
static uint8_t key_val;                 /* 最新按键值 */
static uint8_t setting_buf[4];          /* 温度设定输入缓冲区 */
static uint8_t setting_idx = 0;         /* 当前输入位置 */
static uint32_t last_tick = 0;          /* 上次控制周期时间戳 */
static uint32_t last_uart_tick = 0;     /* 上次串口上报时间戳 */
static uint8_t alarm_active = 0;        /* 报警标志 (防重复触发) */
static uint32_t run_time_ms = 0;        /* 运行时间毫秒累加器 */
static uint8_t ds18b20_conv_started = 0; /* DS18B20 非阻塞转换标志 */
static uint8_t ds18b20_cycle_cnt = 0;   /* DS18B20 读数跳过计数 (每4周期=800ms读一次) */
/* 三段式报警内部计数器 (文件级以便 ProcessKey 复位) */
static uint32_t alarm_startup_cnt = 0;
static float    alarm_min_error   = 0.0f;
static uint32_t alarm_stuck_cnt   = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── 显示启动画面 (2秒) ── */
static void Display_Splash(void)
{
    OLED_Clear();
    OLED_ShowString(16, 0, "Bio-Incubator", 8);
    OLED_ShowString(20, 24, "System v1.0", 6);
    OLED_ShowString(4, 40, "Initializing...", 6);
    OLED_Display();
    HAL_Delay(2000);
}

/* ── 刷新 OLED 显示 ── */
static void Display_Update(void)
{
    OLED_Clear();

    /* 第1行: 标题 */
    OLED_ShowString(0, 0, "Bio-Incubator", 8);

    char line[22];
    RTC_TimeTypeDef rtc_now;

    /* 第2行: 当前温度 */
    snprintf(line, sizeof(line), "Now:%5.1f C", current_temp);
    OLED_ShowString(0, 16, line, 6);

    /* 第3行: 目标温度 */
    snprintf(line, sizeof(line), "Set:%5.1f C", target_temp);
    OLED_ShowString(0, 24, line, 6);

    /* 第4行: 运行状态 */
    switch (sys_state)
    {
        case SYSTEM_STATE_IDLE:
            OLED_ShowString(0, 32, "Status: IDLE", 6);
            break;
        case SYSTEM_STATE_RUNNING:
            OLED_ShowString(0, 32, "Status: RUN", 6);
            break;
        case SYSTEM_STATE_SETTING:
            snprintf(line, sizeof(line), "Set: %d", (int)target_temp);
            OLED_ShowString(0, 40, line, 6);
            OLED_ShowString(0, 48, "Input Temp...", 6);
            OLED_Display();
            return;
        case SYSTEM_STATE_ALARM:
            OLED_ShowString(0, 32, "Status: ALARM", 6);
            break;
    }

    /* 第5行: PID 输出值 */
    snprintf(line, sizeof(line), "PID:%+5.1f%%", (double)pid_out);
    OLED_ShowString(0, 40, line, 6);

    /* 第6行: 系统时间 + 运行时间 */
    HAL_RTC_GetTime(&hrtc, &rtc_now, RTC_FORMAT_BIN);
    snprintf(line, sizeof(line), "%02u:%02u:%02u Run:%4lu s",
        rtc_now.Hours, rtc_now.Minutes, rtc_now.Seconds, run_time_sec);
    OLED_ShowString(0, 48, line, 6);

    OLED_Display();
}

/* ── 温度超限报警检查 (三段式算法) ──
 *
 * 阶段一: 启动免扰 (前10min) → 不报警, 给PID充分时间
 * 阶段二: |偏差|≤2℃ → 正常, 清除报警
 * 阶段三: |偏差|>2℃ 且 持续60s无改善 → 设备可能故障, 报警
 *
 * 传函 G(s)=1.0·e^{-62s}/(225s+1) → τ=62s, T₁=225s
 * 免扰10min = 2.5·T₁, 卡住60s ≈ τ
 */
#define STARTUP_FREE_CYCLES  3000   /* 10min / 0.2s = 3000 控制周期 */
#define STUCK_THRESHOLD      300    /* 60s / 0.2s = 300 周期 */

static void Alarm_ResetCounters(void)
{
    alarm_startup_cnt = 0;
    alarm_min_error   = 0.0f;
    alarm_stuck_cnt   = 0;
}

static void Alarm_ClearBuzzer(void)
{
    if (alarm_active)
    {
        alarm_active = 0;
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
    }
}

static void Alarm_Check(void)
{
    float diff = current_temp - target_temp;
    float abs_err = (diff > 0) ? diff : (-diff);

    /* ── 非运行态: 确保蜂鸣器关闭 (兼容旧版行为) ── */
    if (sys_state != SYSTEM_STATE_RUNNING && sys_state != SYSTEM_STATE_ALARM)
    {
        Alarm_ClearBuzzer();
        return;
    }

    /* ── 阶段一: 启动免扰 ── */
    if (alarm_startup_cnt < STARTUP_FREE_CYCLES)
    {
        alarm_startup_cnt++;
        Alarm_ClearBuzzer();
        return;
    }

    /* ── 阶段二: 偏差在正常范围 ── */
    if (abs_err <= ALARM_TEMP_HIGH)
    {
        alarm_stuck_cnt = 0;
        alarm_min_error = 0.0f;
        if (sys_state == SYSTEM_STATE_ALARM)
            sys_state = SYSTEM_STATE_RUNNING;
        Alarm_ClearBuzzer();
        return;
    }

    /* ── 阶段三: 偏差 > 2℃ ── */
    if (alarm_min_error == 0.0f || abs_err < alarm_min_error)
    {
        alarm_min_error = abs_err;
        alarm_stuck_cnt = 0;
    }
    else
    {
        alarm_stuck_cnt++;
    }

    if (alarm_stuck_cnt >= STUCK_THRESHOLD)
    {
        sys_state = SYSTEM_STATE_ALARM;
        if (!alarm_active)
        {
            alarm_active = 1;
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        }
    }
}

/* ── 按键处理 (状态机) ── */
static void ProcessKey(uint8_t key)
{
    if (key == KEY_NONE) return;

    /* 【*】键: 进入设置 / 退格 */
    if (key == 0x0A)
    {
        if (sys_state == SYSTEM_STATE_SETTING)
        {
            if (setting_idx > 0) setting_idx--;  /* 退格删除一位 */
        }
        else
        {
            sys_state = SYSTEM_STATE_SETTING;    /* 进入温度设置模式 */
            setting_idx = 0;
            memset(setting_buf, 0, sizeof(setting_buf));
        }
        return;
    }

    /* 【#】键: 确认设置 / 启停切换 */
    if (key == 0x0B)
    {
        if (sys_state == SYSTEM_STATE_SETTING)
        {
            if (setting_idx > 0)
            {
                uint16_t raw = 0;
                for (uint8_t i = 0; i < setting_idx; i++)
                {
                    raw = raw * 10 + setting_buf[i];
                }
                /* 2位→整数, 3位→末位为小数 (如 375 → 37.5) */
                float val;
                if (setting_idx >= 3)
                    val = raw / 10.0f;
                else
                    val = (float)raw;

                if (val >= TEMP_MIN && val <= TEMP_MAX)
                {
                    target_temp = val;
                }
            }
            sys_state = SYSTEM_STATE_RUNNING;   /* 确认后开始运行 */
            setting_idx = 0;
            Alarm_ResetCounters();
            alarm_active = 0;
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            PID_Reset(&hpid);
            run_time_sec = 0;
            run_time_ms = 0;
        }
        else
        {
            /* 运行 ↔ 空闲 切换 */
            if (sys_state == SYSTEM_STATE_RUNNING)
            {
                sys_state = SYSTEM_STATE_IDLE;
                Control_Stop(&hctrl);
                run_time_sec = 0;
                run_time_ms = 0;
            }
            else
            {
                sys_state = SYSTEM_STATE_RUNNING;
                Alarm_ResetCounters();
                alarm_active = 0;
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
                run_time_sec = 0;
                run_time_ms = 0;
            }
        }
        return;
    }

    /* 【0-9】键: 在设置模式下输入数字 */
    if (sys_state == SYSTEM_STATE_SETTING && key <= 9)
    {
        if (setting_idx < 4)
        {
            setting_buf[setting_idx++] = key;
        }
    }
}

/* ── 通过串口上报实时数据 ── */
static void UART_SendData(void)
{
    UART_Printf("T:%.1f,SET:%.1f,PID:%.1f,HT:%d%%,CL:%d%%,RUN:%lu\r\n",
        current_temp, target_temp, (double)pid_out,
        hctrl.heat_duty, hctrl.cool_duty, run_time_sec);
}

/* ── 解析并执行上位机串口命令 ── */
static void ProcessUartCmd(void)
{
    if (uart_rx_flag)
    {
        uart_rx_flag = 0;
        char *cmd = (char*)uart_rx_buf;

        if (strncmp(cmd, "SET:", 4) == 0)
        {
            /* 格式: SET:37.5 */
            float val = atof(cmd + 4);
            if (val >= TEMP_MIN && val <= TEMP_MAX)
            {
                target_temp = val;
                PID_Reset(&hpid);
                if (sys_state != SYSTEM_STATE_RUNNING)
                {
                    sys_state = SYSTEM_STATE_RUNNING;
                    run_time_sec = 0;
                }
                UART_Printf("OK:%.1f\r\n", val);
            }
            else
            {
                UART_Printf("ERR:RANGE\r\n");
            }
        }
        else if (strncmp(cmd, "STOP", 4) == 0)
        {
            /* 停止控温 */
            sys_state = SYSTEM_STATE_IDLE;
            Control_Stop(&hctrl);
            run_time_sec = 0;
            run_time_ms = 0;
            UART_Printf("OK:STOPPED\r\n");
        }
        else if (strncmp(cmd, "STATUS", 6) == 0)
        {
            /* 查询当前状态 */
            UART_SendData();
        }
        else
        {
            UART_Printf("ERR:UNKNOWN\r\n");
        }
        uart_rx_len = 0;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
    /* 初始化 RTC 时间: 上电从 00:00:00 开始 */
    RTC_TimeTypeDef rtc_init = {0};
    rtc_init.Hours = 0;
    rtc_init.Minutes = 0;
    rtc_init.Seconds = 0;
    HAL_RTC_SetTime(&hrtc, &rtc_init, RTC_FORMAT_BIN);

    /* 关闭蜂鸣器 (高电平=关) */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);

    /* 初始化 OLED 并显示启动画面 */
    OLED_Init();
    Display_Splash();

    /* 初始化传感器和输入 */
    if (!DS18B20_Init())
    {
        OLED_Clear();
        OLED_ShowString(0, 0, "DS18B20 Error!", 8);
        OLED_ShowString(0, 24, "Check Sensor", 6);
        OLED_Display();
    }
    Key_Init();

    /* 初始化控制模块 */
    PID_Init(&hpid, 15.0f, 0.8f, 2.5f, 0.2f, 100.0f, -100.0f);
    Control_Init();
    UART_Init();

    /* 初始温度读取 (非阻塞: 启动转换后等待完成) */
    DS18B20_StartConversion();
    HAL_Delay(750);
    current_temp = DS18B20_ReadResult();
    ds18b20_conv_started = 0;

    target_temp = TEMP_DEFAULT;
    sys_state = SYSTEM_STATE_IDLE;
    last_tick = HAL_GetTick();
    last_uart_tick = last_tick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        uint32_t now = HAL_GetTick();

        /* ① 按键扫描与处理 (每轮主循环执行, 不依赖 200ms 周期) */
        key_val = Key_Scan();
        if (key_val != KEY_NONE)
        {
            ProcessKey(key_val);
            Display_Update();  /* 按键后立即刷新, 不等 200ms */
        }

        /* 每 200ms 执行一次控制周期 */
        if (now - last_tick >= CONTROL_CYCLE_MS)
        {
            last_tick = now;

            /* ② 温度采集 (非阻塞: 跳过3周期=800ms, 大于DS18B20最大转换时间750ms) */
            if (ds18b20_conv_started)
            {
                ds18b20_cycle_cnt++;
                if (ds18b20_cycle_cnt >= 4)
                {
                    ds18b20_cycle_cnt = 0;
                    current_temp = DS18B20_ReadResult();
                    DS18B20_StartConversion();
                }
            }
            else
            {
                DS18B20_StartConversion();
                ds18b20_conv_started = 1;
                ds18b20_cycle_cnt = 0;
            }

            /* ③ 报警检测 (始终执行, 不依赖运行状态) */
            Alarm_Check();

            /* ④ 运行/报警状态下执行 PID 控制 (报警只响蜂鸣器, 不停控温) */
            if (sys_state == SYSTEM_STATE_RUNNING || sys_state == SYSTEM_STATE_ALARM)
            {
                pid_out = PID_Calculate(&hpid, target_temp, current_temp);
                Control_ProcessOutput(&hctrl, pid_out);

                run_time_ms += CONTROL_CYCLE_MS;
                if (run_time_ms >= 1000)
                {
                    run_time_sec += run_time_ms / 1000;
                    run_time_ms %= 1000;
                }
            }

            /* ⑤ 刷新 OLED 显示 */
            Display_Update();
        }

        /* 每 1s 通过串口上报一次数据 */
        if (now - last_uart_tick >= USB_REPORT_CYCLE)
        {
            last_uart_tick = now;
            UART_SendData();
        }

        /* 接收串口数据并处理上位机命令 */
        UART_ProcessRx();
        ProcessUartCmd();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
