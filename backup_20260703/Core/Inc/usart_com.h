#ifndef __USART_COM_H__
#define __USART_COM_H__

#include "main.h"

/* USART1 缓冲区大小 */
#define UART_TX_BUF_SIZE  128
#define UART_RX_BUF_SIZE  64

/* 初始化 USART1 (PA9=TX, PA10=RX, 115200-8N1) */
void UART_Init(void);
/* 发送原始数据 */
void UART_Send(uint8_t *buf, uint16_t len);
/* 格式化打印 (类似 printf) */
void UART_Printf(const char *fmt, ...);
/* 轮询接收: 每调用一次处理一批收到的字节, 拼成命令后置 uart_rx_flag */
void UART_ProcessRx(void);

/* 接收缓冲区和状态 (volatile, 供外部访问) */
extern volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
extern volatile uint16_t uart_rx_len;
extern volatile uint8_t  uart_rx_flag;

#endif
