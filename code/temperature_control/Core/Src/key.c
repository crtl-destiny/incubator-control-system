/**
 * @file    key.c
 * @brief   4×3 矩阵键盘驱动 (非阻塞)
 *
 * 引脚映射 (GPIOB):
 *   行: PB0, PB1, PB10, PB11  (推挽输出)
 *   列: PB12, PB13, PB14      (上拉输入)
 *
 * 按键布局:
 *   ┌───┬───┬───┐
 *   │ 1 │ 2 │ 3 │
 *   ├───┼───┼───┤
 *   │ 4 │ 5 │ 6 │
 *   ├───┼───┼───┤
 *   │ 7 │ 8 │ 9 │
 *   ├───┼───┼───┤
 *   │ * │ 0 │ # │
 *   └───┴───┴───┘
 *   * = 0x0A (设置/退格), # = 0x0B (确认/启停)
 *
 * 扫描方式: 无阻塞, GPIO 设行后加 ~2µs 等待信号稳定。
 * 去抖: 每检测到一个键后屏蔽 50ms (基于 HAL_GetTick, 不阻塞)。
 * 同键防重: 50ms 窗口内只返回一次相同键值。
 */

#include "key.h"

static const uint8_t key_map[KEY_ROWS][KEY_COLS] = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9},
    {0x0A, 0, 0x0B}
};

static const uint16_t row_pins[KEY_ROWS] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
static const uint16_t col_pins[KEY_COLS] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

void Key_Init(void) { }

uint8_t Key_Scan(void)
{
    static uint8_t last_key = KEY_NONE;
    static uint32_t last_tick = 0;
    uint8_t found = KEY_NONE;

    /* 50ms 去抖窗口: 刚触发过按键则暂不扫描 */
    if (HAL_GetTick() - last_tick < 50) return KEY_NONE;

    for (uint8_t r = 0; r < KEY_ROWS; r++)
    {
        HAL_GPIO_WritePin(GPIOB, row_pins[0] | row_pins[1] | row_pins[2] | row_pins[3], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, row_pins[r], GPIO_PIN_SET);

        /* 等待行信号稳定 (~2µs) */
        for (volatile uint32_t d = 0; d < 200; d++);

        for (uint8_t c = 0; c < KEY_COLS; c++)
        {
            if (HAL_GPIO_ReadPin(GPIOB, col_pins[c]) == GPIO_PIN_SET)
            {
                found = key_map[r][c];
                break;
            }
        }
        if (found != KEY_NONE) break;
    }

    if (found != KEY_NONE && found != last_key)
    {
        last_key = found;
        last_tick = HAL_GetTick();
        return found;
    }

    if (found == KEY_NONE) last_key = KEY_NONE;
    return KEY_NONE;
}

uint8_t Key_GetValue(void)
{
    return Key_Scan();
}
