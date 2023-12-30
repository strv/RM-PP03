/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define USART_RX_BUF_LEN (128)
#define USART_TX_BUF_LEN (256)

const static int USART_RX_BUF_MASK = USART_RX_BUF_LEN - 1;
const static uint32_t ModbusIntervalUs = 3.5f * 10 * 1000 * 1000 / MODBUS_BAUD; // duration of 3.5 character in micro sec
const static uint32_t ModbusInitial = 0xFFFF;
const static uint32_t ModbusPoly = 0xA001;

static uint8_t rx2_buf_[USART_RX_BUF_LEN];
static uint8_t tx2_buf_[USART_TX_BUF_LEN];
static bool tx2_ongoing_ = false;
static int rx2_buf_ri_ = 0;
static int rx2_buf_wi_ = 0;
static uint32_t mb_last_rx_us_ = 0;
static uint32_t mb_timer_ = 0;
static int mb_frame_index_ = 0;
static uint8_t mb_frame_[256];
static int mb_frame_length_ = 0;

static uint16_t mb_crc_reg_ = 0;

static void mb_crc_reset(uint16_t* reg)
{
  *reg = ModbusInitial;
}

static void mb_crc_push(const uint8_t b, uint16_t* reg)
{
  volatile int shifts = 8;
  *reg ^= b;
  while (shifts--)
  {
    *reg >>= 1;
    if (__get_APSR() & APSR_C_Msk)
      *reg ^= ModbusPoly;
  }
}

void usart_init()
{
  LL_USART_ClearFlag_UDR(USART2);
  LL_USART_ClearFlag_TC(USART2);
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableDirectionRx(USART2);
  LL_USART_EnableDirectionTx(USART2);
  LL_USART_EnableDMAReq_TX(USART2);

  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2,
    (uint32_t)LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2,
    (uint32_t)tx2_buf_);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);

  LL_TIM_EnableIT_UPDATE(TIM17);

  LL_USART_Enable(USART2);
  mb_crc_reset(&mb_crc_reg_);
}

void usart2_rx_cb()
{
  mb_last_rx_us_ = mb_timer_;
  const uint8_t c = LL_USART_ReceiveData8(USART2);
  rx2_buf_[rx2_buf_wi_] = c;
  mb_timer_ = 0;
  LL_TIM_EnableCounter(TIM17);
  mb_frame_length_ = 0;
  mb_frame_[mb_frame_index_] = c;
  mb_frame_index_++;

  rx2_buf_wi_ = (rx2_buf_wi_ + 1) & USART_RX_BUF_MASK;
  if (rx2_buf_wi_ == rx2_buf_ri_) rx2_buf_ri_++;
}

void usart2_rx_ore_cb()
{
  mb_timer_ = 0;
}

void usart2_tx_dma_cb()
{
  tx2_ongoing_ = false;
}

int usart2_tx_buf(const uint8_t* pbuf, const int length)
{
  if (length > USART_TX_BUF_LEN)
    return 0;
  memcpy(tx2_buf_, pbuf, length);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, length);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  tx2_ongoing_ = true;
  return length;
}

int usart2_getc()
{
  int c;
  if (rx2_buf_ri_ == rx2_buf_wi_)
    return 0;
  c = rx2_buf_[rx2_buf_ri_++];
  rx2_buf_ri_ &= USART_RX_BUF_MASK;
  return c;
}

void mb_timer_cb()
{
  if (mb_timer_ < ModbusIntervalUs)
  {
    ++mb_timer_;
  }
  else if (mb_timer_ == ModbusIntervalUs)
  {
    LL_USART_DisableIT_RXNE(USART2);
    if (mb_frame_index_ >= 4)  // valid frame length
    {
      uint16_t rx_crc = mb_frame_[mb_frame_index_ - 1] << 8 | mb_frame_[mb_frame_index_ - 2];
      for (int i = 0; i < mb_frame_index_-2; ++i)
      {
        mb_crc_push(mb_frame_[i], &mb_crc_reg_);
      }
      if (rx_crc == mb_crc_reg_)
      {
        mb_frame_length_ = mb_frame_index_;
      }
    }
    mb_crc_reset(&mb_crc_reg_);
    mb_frame_index_ = 0;
    LL_TIM_DisableCounter(TIM17);
    LL_USART_EnableIT_RXNE(USART2);
  }
}

int mb_pop_frame(uint8_t* pframe)
{
  if (mb_frame_length_ == 0)
  {
    return 0;
  }
  LL_USART_DisableIT_RXNE(USART2);
  const int len = mb_frame_length_;
  memcpy(pframe, mb_frame_, mb_frame_length_);
  mb_frame_length_ = 0;
  LL_USART_EnableIT_RXNE(USART2);
  return len;
}

void mb_push_frame(uint8_t* pframe, int length)
{
  if (tx2_ongoing_)
    return;
  const int frame_len = length+2;
  memcpy(tx2_buf_, pframe, length);
  uint16_t reg = 0;
  mb_crc_reset(&reg);
  for (int i = 0; i < length; ++i)
  {
    mb_crc_push(tx2_buf_[i], &reg);
  }
  tx2_buf_[length] = reg & 0xFF;
  tx2_buf_[length+1] = reg >> 8;
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2,
    (uint32_t)tx2_buf_);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, frame_len);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  tx2_ongoing_ = true;
}
/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB7   ------> USART1_RX
  PB6   ------> USART1_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_USART1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART2_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = MODBUS_BAUD;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
