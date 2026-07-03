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
 * 扫描方式: 无阻塞, 每次调用只扫一遍, 靠重复调用自然消抖。
 * 同键防重: 连续两次读到相同键值时自动屏蔽, 松开后才允许再次触发。
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
    uint8_t found = KEY_NONE;

    for (uint8_t r = 0; r < KEY_ROWS; r++)
    {
        HAL_GPIO_WritePin(GPIOB, row_pins[0] | row_pins[1] | row_pins[2] | row_pins[3], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, row_pins[r], GPIO_PIN_SET);

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

    if (found == last_key) return KEY_NONE;
    last_key = found;
    return found;
}

uint8_t Key_GetValue(void)
{
    return Key_Scan();
}
