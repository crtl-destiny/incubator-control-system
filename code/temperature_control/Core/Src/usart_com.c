/**
 * @file    usart_com.c
 * @brief   USART1 串口通信模块 (HAL 驱动)
 *
 * 硬件连接:
 *   PA9  = USART1_TX → USB-to-TTL RXD
 *   PA10 = USART1_RX → USB-to-TTL TXD
 *   GND  → USB-to-TTL GND
 *
 * 配置: 115200-8N1, 由 CubeMX 生成 (MX_USART1_UART_Init)
 * 协议: ASCII 文本, \r\n 换行
 * 接收: 主循环轮询 UART_FLAG_RXNE
 */

#include "usart_com.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include <stdarg.h>
#include <stdio.h>

volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
volatile uint16_t uart_rx_len = 0;
volatile uint8_t  uart_rx_flag = 0;

static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];

void UART_Init(void)
{
    uart_rx_len = 0;
    uart_rx_flag = 0;
}

void UART_Send(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, buf, len, 100);
}

void UART_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char*)uart_tx_buf, UART_TX_BUF_SIZE, fmt, args);
    va_end(args);
    if (len > 0)
        UART_Send(uart_tx_buf, (uint16_t)len);
}

void UART_ProcessRx(void)
{
    if (uart_rx_flag) return;

    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        uint8_t ch = (uint8_t)(huart1.Instance->DR & 0xFF);

        if (ch == '\n')
        {
            uart_rx_buf[uart_rx_len] = '\0';
            uart_rx_flag = 1;
            uart_rx_len = 0;
            break;
        }
        else if (ch != '\r')
        {
            if (uart_rx_len < UART_RX_BUF_SIZE - 1)
                uart_rx_buf[uart_rx_len++] = ch;
        }
    }
}
