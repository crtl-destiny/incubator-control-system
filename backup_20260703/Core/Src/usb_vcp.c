/**
 * @file    usb_vcp.c
 * @brief   USB 虚拟串口 (Virtual COM Port) 通信模块
 *
 * 基于 STM32F103 USB PCD 外设实现 CDC ACM 协议。
 * 端点分配:
 *   EP0      — 控制端点 (设备枚举/描述符)
 *   EP2 IN   — 数据发送 (BULK, 0x82)
 *   EP2 OUT  — 数据接收 (BULK, 0x02)
 *   EP3 INT  — 串口状态通知 (INT, 0x83)
 *
 * 注意: 完整 USB CDC 枚举需要 ST USB Device 中间件
 * (USB_DEVICE_Library) 提供完整的描述符和协议处理。
 * 当前实现提供基础框架, 生产环境需集成中间件。
 */

#include "usb_vcp.h"
#include "stm32f1xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 接收缓冲区和状态标志 (volatile, 在中断中修改) */
volatile uint8_t  usb_rx_buf[USB_RX_BUF_SIZE];
volatile uint16_t usb_rx_len = 0;
volatile uint8_t  usb_rx_flag = 0;

/* 发送缓冲区 (静态, 用于格式化输出) */
static uint8_t usb_tx_buf[USB_TX_BUF_SIZE];

extern PCD_HandleTypeDef hpcd_USB_FS;  /* 在 usb.c 中定义 */

/* CDC 端点参数 */
#define USB_CDC_ACM_PACKET_SIZE   64   /* 数据包最大长度 */
#define CDC_IN_EP                 0x82 /* 数据发送端点 (与配置描述符一致) */
#define CDC_OUT_EP                0x02 /* 数据接收端点 */
#define CDC_CMD_EP                0x83 /* 串口状态通知端点 */

/* 硬件接收缓冲区 */
static uint8_t USB_RxBuffer[USB_CDC_ACM_PACKET_SIZE];

/* USB 描述符 (CDC ACM) */
static const uint8_t USBD_DeviceDesc[18] = {
    0x12,                       /* bLength */
    0x01,                       /* bDescriptorType (DEVICE) */
    0x10, 0x02,                 /* bcdUSB 2.10 */
    0x02,                       /* bDeviceClass (CDC) */
    0x00,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    0x40,                       /* bMaxPacketSize0 */
    0x83, 0x04,                 /* idVendor  (0x0483 = ST) */
    0x40, 0x57,                 /* idProduct (0x5740) */
    0x00, 0x01,                 /* bcdDevice */
    0x01,                       /* iManufacturer */
    0x02,                       /* iProduct */
    0x03,                       /* iSerialNumber */
    0x01                        /* bNumConfigurations */
};

static const uint8_t USBD_ConfigDesc[67] = {
    0x09,                       /* bLength */
    0x02,                       /* bDescriptorType (CONFIGURATION) */
    0x43, 0x00,                 /* wTotalLength (67) */
    0x02,                       /* bNumInterfaces */
    0x01,                       /* bConfigurationValue */
    0x00,                       /* iConfiguration */
    0xC0,                       /* bmAttributes (Self Powered) */
    0x32,                       /* bMaxPower (100mA) */

    /* --- Interface 0: Communication --- */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* CDC Header Functional Descriptor */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* CDC Call Management Functional Descriptor */
    0x05, 0x24, 0x01, 0x01, 0x01,
    /* CDC ACM Functional Descriptor */
    0x04, 0x24, 0x02, 0x02,
    /* CDC Union Functional Descriptor */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* Endpoint: Interrupt IN (notify) */
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x0A,

    /* --- Interface 1: Data --- */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* Endpoint: Bulk OUT */
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    /* Endpoint: Bulk IN */
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
};

static const uint8_t USBD_StrDesc[][32] = {
    {0x04, 0x03, 0x09, 0x04},           /* LANGID: English */
    {0x1C, 0x03, 'S',0,'T',0,'M',0,'3',0,'2',0,' ',0,'C',0,'D',0,'C',0},
    {0x1C, 0x03, 'B',0,'i',0,'o',0,'-',0,'I',0,'n',0,'c',0,'u',0,'b',0,'a',0,'t',0,'o',0,'r',0},
    {0x1A, 0x03, '1',0,'.',0,'0',0,'.',0,'0',0},
};

/* 初始化 USB 设备: 配置 PMA 缓冲区并启动 */
void USB_VCP_Init(void)
{
    /* 配置 USB PMA (Packet Memory Area) 缓冲区地址
       所有缓冲区从 0x40 起步, 避免与 BTABLE (0x00~0x3F) 重叠 */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, 0x40);  /* EP0 OUT */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, 0x80);  /* EP0 IN */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_OUT_EP, PCD_SNG_BUF, 0xC0);  /* EP2 OUT */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_IN_EP,  PCD_SNG_BUF, 0x100); /* EP2 IN */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x140); /* EP3 INT */

    HAL_PCD_Start(&hpcd_USB_FS);
}

/* 通过 USB 发送数据 (自动分片为 64 字节包) */
void USB_VCP_Send(uint8_t *buf, uint16_t len)
{
    uint16_t sent = 0;
    while (sent < len)
    {
        uint16_t chunk = (len - sent > USB_CDC_ACM_PACKET_SIZE)
                         ? USB_CDC_ACM_PACKET_SIZE : (len - sent);
        HAL_PCD_EP_Transmit(&hpcd_USB_FS, CDC_IN_EP, buf + sent, chunk);
        HAL_Delay(1);  /* 等待 USB 硬件完成上一包发送 */
        sent += chunk;
    }
}

/* 格式化打印 (printf 风格) */
void USB_VCP_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char*)usb_tx_buf, USB_TX_BUF_SIZE, fmt, args);
    va_end(args);
    if (len > 0)
    {
        USB_VCP_Send(usb_tx_buf, (uint16_t)len);
    }
}

/* 查询接收缓冲区中的数据量 */
uint16_t USB_VCP_Available(void)
{
    return usb_rx_len;
}

/* 从接收缓冲区读取一个字节 */
uint8_t USB_VCP_ReadByte(void)
{
    uint8_t byte = 0;
    if (usb_rx_len > 0)
    {
        byte = usb_rx_buf[0];
        usb_rx_len--;
        memmove((void*)usb_rx_buf, (void*)(usb_rx_buf + 1), usb_rx_len);
    }
    return byte;
}

/* 从接收缓冲区读取多个字节 */
void USB_VCP_ReadBuffer(uint8_t *buf, uint16_t len)
{
    uint16_t copy_len = (len < usb_rx_len) ? len : (uint16_t)usb_rx_len;
    memcpy(buf, (void*)usb_rx_buf, copy_len);
    usb_rx_len -= copy_len;
    memmove((void*)usb_rx_buf, (void*)(usb_rx_buf + copy_len), usb_rx_len);
}

/* ========== USB PCD 回调函数 ========== */

/**
 * USB 控制传输 (Setup Stage) 回调
 * 处理 USB 标准请求和 CDC 类请求
 */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    /* STM32F1 的 hpcd->Setup 是 uint32_t[12], 用 memcpy 提取 8 字节 setup packet */
    uint8_t req[8];
    memcpy(req, hpcd->Setup, 8);
    uint8_t  bmRequestType = req[0];
    uint8_t  bRequest      = req[1];
    uint16_t wValue        = (uint16_t)(req[2] | (req[3] << 8));
    uint16_t wLength       = (uint16_t)(req[6] | (req[7] << 8));

    /* ── 标准请求 (bmRequestType bits 5:4 = 00) ── */
    if ((bmRequestType & 0x60) == 0x00)
    {
        switch (bRequest)
        {
            case 0x05:  /* SET_ADDRESS */
                HAL_PCD_SetAddress(hpcd, (uint8_t)(wValue & 0x7F));
                break;
            case 0x06:  /* GET_DESCRIPTOR */
            {
                uint8_t type = (uint8_t)(wValue >> 8);
                uint8_t idx  = (uint8_t)(wValue & 0xFF);

                if (type == 0x01) { /* 设备描述符 */
                    uint16_t len = (wLength < 18) ? wLength : 18;
                    HAL_PCD_EP_Transmit(hpcd, 0x80, (uint8_t*)USBD_DeviceDesc, len);
                } else if (type == 0x02) { /* 配置描述符 */
                    uint16_t len = (wLength < 67) ? wLength : 67;
                    HAL_PCD_EP_Transmit(hpcd, 0x80, (uint8_t*)USBD_ConfigDesc, len);
                } else if (type == 0x03) { /* 字符串描述符 */
                    if (idx < 4) {
                        const uint8_t *str = USBD_StrDesc[idx];
                        uint16_t len = (wLength < str[0]) ? wLength : str[0];
                        HAL_PCD_EP_Transmit(hpcd, 0x80, (uint8_t*)str, len);
                    } else {
                        HAL_PCD_EP_SetStall(hpcd, 0x80);
                    }
                } else {
                    HAL_PCD_EP_SetStall(hpcd, 0x80);
                }
                break;
            }
            case 0x09:  /* SET_CONFIGURATION */
                break;
            default:
                HAL_PCD_EP_SetStall(hpcd, 0x80);
                HAL_PCD_EP_SetStall(hpcd, 0x00);
                break;
        }
    }
    /* ── CDC 类请求 (bmRequestType bits 5:4 = 01) ── */
    else if ((bmRequestType & 0x60) == 0x20)
    {
        if (bRequest == 0x20 || bRequest == 0x22 || bRequest == 0x21)
        {
            /* CDC 类请求无需特殊操作, 回复 ZLP 即可 */
        }
        else
        {
            HAL_PCD_EP_SetStall(hpcd, 0x80);
            HAL_PCD_EP_SetStall(hpcd, 0x00);
        }
    }
    else
    {
        HAL_PCD_EP_SetStall(hpcd, 0x80);
        HAL_PCD_EP_SetStall(hpcd, 0x00);
    }
}

/**
 * USB 数据接收 (Data OUT Stage) 回调
 * 将收到的数据拷贝到 usb_rx_buf 并置标志 (不再在此处理命令)
 */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (epnum == (CDC_OUT_EP & 0x7F))
    {
        uint16_t len = HAL_PCD_EP_GetRxCount(hpcd, epnum);
        if (len > 0 && len <= USB_RX_BUF_SIZE)
        {
            memcpy((void*)usb_rx_buf, USB_RxBuffer, len);
            usb_rx_len = len;
            usb_rx_flag = 1;
        }
        HAL_PCD_EP_Receive(hpcd, CDC_OUT_EP, USB_RxBuffer, USB_CDC_ACM_PACKET_SIZE);
    }
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    /* 数据发送完成，无需额外处理 */
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    /* 帧起始中断，暂不使用 */
}

/**
 * USB 总线复位回调
 * 重新打开所有端点并准备接收
 */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    /* STM32F1: HAL_PCD_EP_Open(hpcd, ep_addr, ep_mps, ep_type) */
    HAL_PCD_EP_Open(hpcd, 0x00, 64, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(hpcd, 0x80, 64, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(hpcd, CDC_OUT_EP, USB_CDC_ACM_PACKET_SIZE, EP_TYPE_BULK);
    HAL_PCD_EP_Open(hpcd, CDC_IN_EP,  USB_CDC_ACM_PACKET_SIZE, EP_TYPE_BULK);
    HAL_PCD_EP_Open(hpcd, CDC_CMD_EP, 8, EP_TYPE_INTR);
    HAL_PCD_EP_Receive(hpcd, CDC_OUT_EP, USB_RxBuffer, USB_CDC_ACM_PACKET_SIZE);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) { }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)  { }
