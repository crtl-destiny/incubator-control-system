#ifndef __USB_VCP_H__
#define __USB_VCP_H__

#include "main.h"

/* USB 虚拟串口缓冲区大小 */
#define USB_TX_BUF_SIZE  128
#define USB_RX_BUF_SIZE  64

/* 初始化USB设备并启动 */
void USB_VCP_Init(void);
/* 发送原始数据 */
void USB_VCP_Send(uint8_t *buf, uint16_t len);
/* 格式化打印 (类似printf) */
void USB_VCP_Printf(const char *fmt, ...);

/* 查询是否有接收数据 */
uint16_t USB_VCP_Available(void);
/* 读取一个字节 */
uint8_t USB_VCP_ReadByte(void);
/* 读取多个字节 */
void USB_VCP_ReadBuffer(uint8_t *buf, uint16_t len);

/* 接收缓冲区和状态 (volatile, 供外部访问) */
extern volatile uint8_t  usb_rx_buf[USB_RX_BUF_SIZE];
extern volatile uint16_t usb_rx_len;
extern volatile uint8_t  usb_rx_flag;

#endif
