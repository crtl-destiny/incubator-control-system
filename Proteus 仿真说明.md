# Proteus 仿真接线图

## STM32F103C8T6 引脚连接

```
                         ┌─────────────────────────────────────┐
                         │          STM32F103C8T6              │
                         │                                     │
     DS18B20 ── PA0 ────┤ GPIO                               │
                         │                                     │
     OLED_SCL ── PB6 ───┤ I2C1_SCL                            │
     OLED_SDA ── PB7 ───┤ I2C1_SDA                            │
                         │                                     │
     KEY_R0 ──── PB0 ───┤ GPIO (行)                           │
     KEY_R1 ──── PB1 ───┤ GPIO                                │
     KEY_R2 ─── PB10 ───┤ GPIO                                │
     KEY_R3 ─── PB11 ───┤ GPIO                                │
                         │                                     │
     KEY_C0 ─── PB12 ───┤ GPIO (列, 上拉输入)                 │
     KEY_C1 ─── PB13 ───┤ GPIO                                │
     KEY_C2 ─── PB14 ───┤ GPIO                                │
                         │                                     │
     BTS7960_IN1 ─ PA6 ─┤ TIM3_CH1 (PWM)                      │
     BTS7960_IN2 ─ PA7 ─┤ TIM3_CH2 (PWM)                      │
                         │                                     │
     BUZZER ──── PA8 ───┤ GPIO (低电平有效)                   │
                         │                                     │
     USART1_TX ─ PA9 ───┤ USART1                              │
     USART1_RX ─ PA10 ──┤ USART1                              │
                         │                                     │
     SWDIO ──── PA13 ───┤ SWD                                 │
     SWCLK ──── PA14 ───┤ SWD                                 │
                         │                                     │
                         └─────────────────────────────────────┘
```

## Proteus 元件清单

| 元件 | Proteus 型号 | 说明 |
|------|-------------|------|
| MCU | STM32F103C8T6 | 或 STM32F103R6 (Proteus 可能没有 C8T6) |
| 温度传感器 | DS18B20 | 或用电位器模拟电压输入后软件转换 |
| OLED | — | Proteus 无 SSD1306 模型, 用 **Virtual Terminal** 替代显示 |
| 键盘 | 4×3 BUTTON 矩阵 | 或用独立按键 + 电阻网络 |
| BTS7960 | — | 用 **PWM 电压源** 观察波形 |
| 蜂鸣器 | BUZZER / LED | 用 LED 替代观察 |
| USART1 | VIRTUAL TERMINAL | 串口调试 |
| RTC | — | 用 **LCD** 显示时间 |

## 关键注意事项

### 1. OLED 替代方案

Proteus 没有 SSD1306 OLED 模型。用 **Virtual Terminal** 截取显示内容：

```
STM32 PB6 ─── VIRTUAL TERMINAL (I2C 调试)
```

或在代码中加入串口输出日志，通过 USART1 Virtual Terminal 查看系统状态。

### 2. 系统时钟

Proteus 仿真中 **HSE 8MHz** 可能无法起振。改用 HSI 内部时钟：

```c
// 在 SystemClock_Config() 中改为 HSI:
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; // HSI 8MHz /2 = 4MHz → PLLx16 = 64MHz
RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
```

或直接外部设置晶振属性为 8MHz。

### 3. RTC

Proteus 不支持 LSE 32.768kHz 仿真。将 RTC 时钟源改为 **LSI**（约 40kHz 内部 RC），或用软件计数器代替 RTC。

### 4. 虚拟串口 (USART1)

用 Proteus 的 **VIRTUAL TERMINAL** 连接 PA9/PA10：

```
PA9 (TX) ─── VIRTUAL TERMINAL RXD
PA10 (RX) ─── VIRTUAL TERMINAL TXD
```

波特率设为 115200，可查看数据上报和发送命令。

## 最小可仿真配置

```
                        ┌──────────────┐
 PA0 ───────── 电位器    │  模拟 DS18B20 │
                        └──────────────┘
                     
                        ┌──────────────┐
 PB6 ───── VIRTUAL TERM│  OLED 调试    │
 PB7 ───── VIRTUAL TERM│  (I2C 日志)   │
                        └──────────────┘

                        ┌──────────────┐
 PA6 ───── OSCILLOSCOPE │  BTS7960 PWM  │
 PA7 ───── OSCILLOSCOPE │  (加热/制冷)  │
                        └──────────────┘

                        ┌──────────────┐
 PA9 ───── VIRTUAL TERM│  USART1 命令  │
 PA10 ──── VIRTUAL TERM│  + 数据上报    │
                        └──────────────┘

 PB12~14 ── 按键矩阵     │  输入控制     │
 PB0/PB1/PB10/PB11 ──   │  行输出       │
                        └──────────────┘
```

## Proteus .hex 文件加载

编译 Keil 项目后在 `MDK-ARM/temperature_control.axf` 路径。Proteus 中 MCU 属性 → Program File → 加载 `.axf` 或 `.hex` 文件（需 Keil 设置输出 .hex：Target → Create HEX File ✅）。