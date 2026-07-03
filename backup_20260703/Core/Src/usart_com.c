/**
 * @file    usart_com.c
 * @brief   USART1 串口通信模块 (CMSIS 寄存器级驱动)
 *
 * 硬件连接:
 *   PA9  = USART1_TX → USB-to-TTL RXD
 *   PA10 = USART1_RX → USB-to-TTL TXD
 *   GND  → USB-to-TTL GND
 *
 * 配置: 115200 baud, 8N1
 * 协议: ASCII 文本, \r\n 换行
 * 接收: 主循环周期性轮询 USART_SR_RXNE
 */

#include "usart_com.h"
#include "stm32f1xx_hal.h"
#include <stdarg.h>
#include <stdio.h>

volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
volatile uint16_t uart_rx_len = 0;
volatile uint8_t  uart_rx_flag = 0;

static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];

void UART_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* PA9 = USART1_TX (复用推挽, 50MHz) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA10 = USART1_RX (上拉输入) */
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* BRR: 72000000 / (16 * 115200) = 39.0625 → Mantissa=39, Fraction=1 */
    USART1->BRR = (39 << 4) | 1;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART_Send(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = buf[i];
    }
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

    while (USART1->SR & USART_SR_RXNE)
    {
        uint8_t ch = (uint8_t)(USART1->DR & 0xFF);

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
