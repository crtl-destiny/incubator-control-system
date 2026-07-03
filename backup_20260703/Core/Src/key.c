/**
 * @file    key.c
 * @brief   4×3 矩阵键盘驱动
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
 */

#include "key.h"

/* 按键编码映射表: [行][列] → 键值 */
static const uint8_t key_map[KEY_ROWS][KEY_COLS] = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9},
    {0x0A, 0, 0x0B}
};

/* 行引脚和列引脚定义 (对应 CubeMX 配置) */
static const uint16_t row_pins[KEY_ROWS] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
static const uint16_t col_pins[KEY_COLS] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

void Key_Init(void)
{
    /* GPIO 已在 MX_GPIO_Init() 中完成初始化, 此处无需额外操作 */
}

/**
 * 扫描矩阵键盘 (逐行扫描法)
 * @return 按键编码, 无按键时返回 KEY_NONE (0xFF)
 */
uint8_t Key_Scan(void)
{
    for (uint8_t r = 0; r < KEY_ROWS; r++)
    {
        /* 所有行输出低电平, 然后单独拉高当前行 */
        HAL_GPIO_WritePin(GPIOB, row_pins[0] | row_pins[1] | row_pins[2] | row_pins[3], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, row_pins[r], GPIO_PIN_SET);

        /* 读取列引脚, 检测是否有按键按下 */
        for (uint8_t c = 0; c < KEY_COLS; c++)
        {
            if (HAL_GPIO_ReadPin(GPIOB, col_pins[c]) == GPIO_PIN_SET)
            {
                /* 消抖延时 */
                HAL_Delay(10);
                /* 等待按键释放 */
                while (HAL_GPIO_ReadPin(GPIOB, col_pins[c]) == GPIO_PIN_SET);
                HAL_Delay(10);
                return key_map[r][c];
            }
        }
    }
    return KEY_NONE;
}

uint8_t Key_GetValue(void)
{
    return Key_Scan();
}
