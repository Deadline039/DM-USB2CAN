//
// Created by Deadline039 on 3/3/2025.
//

#include "dm_protocol.h"

#include "usart.h"
#include "usbd_cdc_if.h"

#include "can.h"
#define CAN_SEND_TIMEOUT 1000
#define LED_TOGGLE_TIMEGAP_MS 25

#include <stdbool.h>

/* 发送次数 */
uint32_t g_send_times;
/* 发送时间间隔 */
uint32_t g_send_time_gap;

static CAN_TxHeaderTypeDef CAN_TxHeader;

/* CAN 发送数据内容, 用联合体方便自增 */
static union
{
  uint8_t data[8];
  uint64_t number;
} CAN_TxData;

static bool idAcc = false;
static bool dataAcc = false;

/**
 * @brief 命令枚举
 */
typedef enum __packed
{
  DM_HEARTBEAT = 0x00,
  DM_RX_FAILED = 0x01,
  DM_RX_SUCCESS = 0x11,
  DM_TX_FAILED = 0x02,
  DM_TX_SUCCESS = 0x12,
} DM_Report_Cmd_t;

/**
 * @brief 往上位机上报的数据帧格式
 */
typedef struct __packed
{
  uint8_t header;         /*!< 帧头 0xAA */
  DM_Report_Cmd_t cmd;    /*!< 指令 */
  uint8_t dataLength : 6; /*!< 数据长度 */
  uint8_t idType : 1;     /*!< ID 标识: 0-标准帧; 1-扩展帧*/
  uint8_t dataType : 1;   /*!< 数据标识: 0-数据帧; 1-远程帧*/
  uint32_t id;            /*!< ID 号 */
  uint8_t data[8];        /*!< 数据 */
  uint8_t end;            /*!< 帧尾 0x55 */
} DM_USB_ReportData_t;

static DM_USB_ReportData_t USB_ReportData = {
    .header = 0xAA, .end = 0x55};

static CAN_RxHeaderTypeDef CAN_RxHeader;
static uint8_t CAN_RxData[8];

static const uint32_t CAN_BaudRateConfig[] = {
    0x001c0004, /* 1000 kbps */
    0x00070009, /* 800 kbps */
    0x0005000e, /* 666.6 kbps */
    0x001c0009, /* 500 kbps */
    0x00070013, /* 400 kbps */
    0x001c0013, /* 250 kbps */
    0x001c0018, /* 200 kbps */
    0x001c0027, /* 125 kbps */
    0x001c0031, /* 100 kbps */
    0x00070063, /* 80 kbps */
    0x001c0063, /* 50 kbps */
    0x001c007c, /* 40 kbps */
    0x001c00f9, /* 20 kbps */
    0x001c01f3, /* 10 kbps */
    0x001c03e7, /* 5 kbps */
};

/**
 * @brief 设置 CAN 波特率
 *
 * @param index 波特率索引
 */
static void USB_CAN_SetBaudRate(uint8_t index)
{
  if (index >= sizeof(CAN_BaudRateConfig) / sizeof(CAN_BaudRateConfig[0]))
  {
    return;
  }

  HAL_CAN_Init(&hcan1);
  CAN1->BTR = CAN_BaudRateConfig[index];
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}

/**
 * @brief 向上位机上报 CAN 消息
 *
 */
static void USB_Report_RxMessage(void)
{
  USB_ReportData.cmd = DM_RX_SUCCESS;
  USB_ReportData.dataLength = CAN_RxHeader.DLC;
  USB_ReportData.idType = CAN_RxHeader.IDE == CAN_ID_STD ? 0 : 1;
  USB_ReportData.dataType = CAN_RxHeader.RTR == CAN_RTR_DATA ? 0 : 1;
  USB_ReportData.id = CAN_RxHeader.IDE == CAN_ID_STD ? CAN_RxHeader.StdId : CAN_RxHeader.ExtId;
  memcpy(&USB_ReportData.data, &CAN_RxData, CAN_RxHeader.DLC);

  CDC_Transmit_FS((uint8_t *)&USB_ReportData, sizeof(USB_ReportData));
}

/**
 * @brief 向上位机上报接收 CAN 失败
 *
 */
static void USB_Report_RxFailed(void)
{
  USB_ReportData.cmd = DM_RX_FAILED;

  CDC_Transmit_FS((uint8_t *)&USB_ReportData, sizeof(USB_ReportData));
}

/**
 * @brief 向上位机上报发送 CAN 消息完成
 *
 */
static void USB_Report_TxSuccess(void)
{
  static uint32_t lastToggleTime = 0;
  USB_ReportData.cmd = DM_TX_SUCCESS;
  USB_ReportData.dataLength = CAN_TxHeader.DLC;
  USB_ReportData.idType = CAN_TxHeader.IDE == CAN_ID_STD ? 0 : 1;
  USB_ReportData.dataType = CAN_TxHeader.RTR == CAN_RTR_DATA ? 0 : 1;
  USB_ReportData.id = CAN_TxHeader.IDE == CAN_ID_STD ? CAN_TxHeader.StdId : CAN_TxHeader.ExtId;
  memcpy(&USB_ReportData.data, &CAN_TxData, CAN_TxHeader.DLC);

  CDC_Transmit_FS((uint8_t *)&USB_ReportData, sizeof(USB_ReportData));

  if (HAL_GetTick() - lastToggleTime > LED_TOGGLE_TIMEGAP_MS)
  {
    HAL_GPIO_TogglePin(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin);
    lastToggleTime = HAL_GetTick();
  }
}

/**
 * @brief 向上位机上报发送 CAN 消息失败
 *
 */
static void USB_Report_TxFailed(void)
{
  USB_ReportData.cmd = DM_TX_FAILED;
  USB_ReportData.dataLength = CAN_TxHeader.DLC;
  USB_ReportData.idType = CAN_TxHeader.IDE == CAN_ID_STD ? 0 : 1;
  USB_ReportData.dataType = CAN_TxHeader.RTR == CAN_RTR_DATA ? 0 : 1;
  USB_ReportData.id = CAN_TxHeader.IDE == CAN_ID_STD ? CAN_TxHeader.StdId : CAN_TxHeader.ExtId;
  memcpy(&USB_ReportData.data, &CAN_TxData, CAN_TxHeader.DLC);

  CDC_Transmit_FS((uint8_t *)&USB_ReportData, sizeof(USB_ReportData));
}

/**
 * @brief 向上位机上报心跳
 */
static void USB_Report_Heartbeat(void)
{
  USB_ReportData.cmd = DM_HEARTBEAT;
  CDC_Transmit_FS((uint8_t *)&USB_ReportData, sizeof(DM_USB_ReportData_t));
}


/**
 * @brief 解析 CAN 发送帧
 *
 * @param USB_Data USB 接收到的数据
 */
static void USB_CAN_ParseFrame(uint8_t *USB_Data)
{

  if (USB_Data[3] == 0x02)
  {
    USB_Report_Heartbeat();
    return;
  }

  memcpy(&g_send_times, &USB_Data[4], sizeof(g_send_times));
  memcpy(&g_send_time_gap, &USB_Data[8], sizeof(g_send_time_gap));

  /* 除以 10 才是毫秒 */
  g_send_time_gap /= 10;

  CAN_TxHeader.IDE = USB_Data[12] == 0 ? CAN_ID_STD : CAN_ID_EXT;
  if (CAN_TxHeader.IDE == CAN_ID_STD)
  {
    memcpy(&CAN_TxHeader.StdId, &USB_Data[13], sizeof(CAN_TxHeader.StdId));
  }
  else
  {
    memcpy(&CAN_TxHeader.ExtId, &USB_Data[13], sizeof(CAN_TxHeader.ExtId));
  }

  CAN_TxHeader.RTR = USB_Data[17] == 0 ? CAN_RTR_DATA : CAN_RTR_REMOTE;
  CAN_TxHeader.DLC = USB_Data[18];

  idAcc = USB_Data[19];
  dataAcc = USB_Data[20];

  memcpy(&CAN_TxData, &USB_Data[21], sizeof(CAN_TxData));

}


/**
 * @brief 发送 CAN 数据
 *
 */
void DM_USB_SendCAN_Message(void)
{
  uint32_t waitTime = 0;
  uint32_t txMailbox = CAN_TX_MAILBOX0;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
    ++waitTime;

    if (waitTime > CAN_SEND_TIMEOUT)
    {
      USB_Report_TxFailed();
      return;
    }
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &CAN_TxHeader, (uint8_t *)&CAN_TxData, &txMailbox) != HAL_OK)
  {
    USB_Report_TxFailed();
    return;
  }

  USB_Report_TxSuccess();

  if (idAcc)
  {
    if (CAN_TxHeader.IDE == CAN_ID_STD)
    {
      ++CAN_TxHeader.StdId;
    }
    else
    {
      ++CAN_TxHeader.ExtId;
    }
  }

  if (dataAcc)
  {
    ++CAN_TxData.number;
  }
}

/**
 * @brief 处理从 USB 虚拟串口收到的数据
 *
 * @param data 数据
 * @param len 长度
 */
void DM_USB_Process(uint8_t *data, uint32_t len)
{
  UART_TxWrite(data, len);

  if (data[0] != 0x55)
  {
    return;
  }

  switch (data[1])
  {
  case 0xAA:
    /* 发送 CAN 数据帧 */
    USB_CAN_ParseFrame(data);
    break;

  case 0x03:
    /* 停止持续发送 CAN 数据帧 */
    g_send_times = 0;
    break;

  case 0x04:
    /* PC 与设备心跳 */
    if (data[3] == 0xAA && data[4] == 0x55)
    {
      USB_Report_Heartbeat();
    }
    break;

  case 0x05:
    /* 设置 CAN 波特率 */
    if (data[3] == 0xAA && data[4] == 0x55)
    {
      USB_CAN_SetBaudRate(data[2]);
    }
    break;

  default:
    break;
  }

}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  static uint32_t lastToggleTime = 0;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData) == HAL_OK)
  {
    USB_Report_RxMessage();
  }
  else
  {
    USB_Report_RxFailed();
  }

  if (HAL_GetTick() - lastToggleTime > LED_TOGGLE_TIMEGAP_MS)
  {
    HAL_GPIO_TogglePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);
    lastToggleTime = HAL_GetTick();
  }
}