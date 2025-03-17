/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "ring_fifo.h"


typedef struct
{
  struct ring_fifo_t *rx_fifo;
  uint8_t rx_fifo_buf[16384];
  uint32_t head_ptr;

  struct ring_fifo_t *tx_fifo;
  uint8_t tx_fifo_buf[16384];

  volatile uint32_t tc_flag;
} UART_FIFO_t;

static UART_FIFO_t uart_rt;

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  uart_rt.rx_fifo = ring_fifo_init(uart_rt.rx_fifo_buf,
                                   sizeof(uart_rt.rx_fifo_buf) / sizeof(uart_rt.rx_fifo_buf[0]),
                                   RF_TYPE_STREAM);
  uart_rt.tx_fifo = ring_fifo_init(uart_rt.tx_fifo_buf,
                                   sizeof(uart_rt.tx_fifo_buf) / sizeof(uart_rt.tx_fifo_buf[0]),
                                   RF_TYPE_STREAM);
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */
    uart_rt.head_ptr = 0;
    uart_rt.tc_flag = 1;

    uart_rt.tx_fifo->head = 0;
    uart_rt.tx_fifo->tail = 0;

    uart_rt.rx_fifo->head = 0;
    uart_rt.rx_fifo->tail = 0;


    /* USER CODE END USART2_MspInit 0 */

    /** Initializes the peripherals clock
    */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart2_tx);

    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspInit 1 */

    /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspDeInit 0 */

    /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspDeInit 1 */

    /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static inline uint32_t UART_WriteRxFIFO(const void *data, uint32_t len)
{
  uint32_t copied;
  if ((NULL == data) || (0 == len)) { return 0; }

  copied = ring_fifo_write(uart_rt.rx_fifo, data, len);
  return copied;
}

uint32_t UART_Read(void *buf, uint32_t len)
{
  if ((NULL == buf) || (0 == len)) { return 0; }

  return ring_fifo_read(uart_rt.rx_fifo, buf, len);
}

uint32_t UART_TxWrite(const void *data, uint32_t len)
{
  uint32_t copied;
  if ((NULL == data) || (0 == len)) { return 0; }

  copied = ring_fifo_write(uart_rt.tx_fifo, data, len);
  return copied;
}

static inline uint32_t uart1_read_tx_fifo(void *buf, uint32_t len)
{
  if ((NULL == buf) || (0 == len)) { return 0; }

  return ring_fifo_read(uart_rt.tx_fifo, buf, len);
}


void UART_TxSend(void)
{
  uint32_t len;
  /* Notice static */
  static uint8_t buf[4096];

  if (0 == uart_rt.tc_flag)
  {
    return;
  }

  len = uart1_read_tx_fifo(buf, sizeof(buf));
  if (len > 0)
  {
    uart_rt.tc_flag = 0;
    HAL_UART_Transmit_DMA(&huart2, buf, len);
  }
}


/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t tail_ptr;
  uint32_t offset, copy;

  /*
   * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
   * |                  half                  |
   * |                    | head_ptr tail_ptr |
   * |                    |    |            | |
   * |                    v    v            v |
   * | ------------------------************** |
   * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
   */

  tail_ptr = huart->RxXferSize;

  offset = uart_rt.head_ptr % huart->RxXferSize;
  copy = tail_ptr - offset;
  uart_rt.head_ptr += copy;

  UART_WriteRxFIFO(huart->pRxBuffPtr + offset, copy);

}

/**
  * @brief  Rx Half Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t tail_ptr;
  uint32_t offset, copy;

  /*
   * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
   * |                  half                  |
   * |     head_ptr   tail_ptr                |
   * |         |          |                   |
   * |         v          v                   |
   * | --------*******************----------- |
   * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
   */

  tail_ptr = (huart->RxXferSize >> 1) + (huart->RxXferSize & 1);

  offset = uart_rt.head_ptr % huart->RxXferSize;
  copy = tail_ptr - offset;
  uart_rt.head_ptr += copy;

  UART_WriteRxFIFO(huart->pRxBuffPtr + offset, copy);
}

void UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
  uint32_t tail_ptr;
  uint32_t copy, offset;

  /*
   * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
   * |     head_ptr          tail_ptr         |
   * |         |                 |            |
   * |         v                 v            |
   * | --------*******************----------- |
   * +~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~+
   */

  /* 已接收 */
  tail_ptr = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);

  offset = uart_rt.head_ptr % huart->RxXferSize;
  copy = tail_ptr - offset;
  uart_rt.head_ptr += copy;

  UART_WriteRxFIFO(huart->pRxBuffPtr + offset, copy);

}

/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  __IO uint32_t tmpErr = 0x00U;

  tmpErr = HAL_UART_GetError(huart);
  if (HAL_UART_ERROR_NONE == tmpErr)
  {
    return;
  }

  switch (tmpErr)
  {
  case HAL_UART_ERROR_PE:
    __HAL_UART_CLEAR_PEFLAG(huart);
    break;
  case HAL_UART_ERROR_NE:
    __HAL_UART_CLEAR_NEFLAG(huart);
    break;
  case HAL_UART_ERROR_FE:
    __HAL_UART_CLEAR_FEFLAG(huart);
    break;
  case HAL_UART_ERROR_ORE:
    __HAL_UART_CLEAR_OREFLAG(huart);
    break;
  case HAL_UART_ERROR_DMA:

    break;
  default:
    break;
  }

  if (NULL != huart->hdmarx)
  {
    while (HAL_UART_Receive_DMA(huart, huart->pRxBuffPtr, huart->RxXferSize))
    {
      __HAL_UNLOCK(huart);
    }
  }
  else
  {
    /* 恢复接收地址指针到初始 buffer 位置 ，初始地址 = 当前地址 - 已接收的数据个数，已接收的数据个数 = 需要接收数 - 还未接收数*/
    while (HAL_UART_Receive_IT(huart, huart->pRxBuffPtr - (huart->RxXferSize - huart->RxXferCount), huart->RxXferSize))
    {
      __HAL_UNLOCK(huart);
    }
  }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_rt.tc_flag = 1;
}

/* USER CODE END 1 */