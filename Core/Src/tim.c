/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
static PWM_DIR dir_ = 0;
static uint32_t oc_high_ = 0;
static uint32_t oc_low_ = 0;
static uint32_t oc_cl_ = 0;
static uint32_t oc_si_ = 0;
static uint32_t superimpose_cycle_ = 0;
static uint32_t superimpose_cnt_ = 0;
static uint32_t superimpose_compare_ = 0;
static bool enabled_ = true;

void pwm_init(void)
{
  LL_TIM_ClearFlag_UPDATE(TIM1);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);
}

void pwm_set_constant_light_rate(const uint16_t rate_cl)
{
  oc_cl_ = rate_cl * PWM_CYCLE / UINT16_MAX;
}

void pwm_set_supoerimpose_amplitude(const uint16_t rate_superimpose)
{
  oc_si_ = (rate_superimpose > UINT16_MAX/2 ? UINT16_MAX/2 : rate_superimpose) * (PWM_CYCLE - oc_cl_) / UINT16_MAX;
}

void pwm_set_rate(const uint16_t rate, const PWM_DIR dir)
{
  int32_t high, low;
  high = rate * (PWM_CYCLE - oc_cl_) / UINT16_MAX + oc_cl_ + oc_si_ / 2;
  low = high - oc_si_;

  if (low < oc_cl_)
  {
    high += oc_cl_ - low;
    low = oc_cl_;
  }
  if (high > PWM_CYCLE)
  {
    low += high - PWM_CYCLE;
    high = PWM_CYCLE;
  }

  // critical section
  LL_TIM_DisableIT_UPDATE(TIM1);
  dir_ = dir;
  oc_low_ = low;
  oc_high_ = high;
  LL_TIM_EnableIT_UPDATE(TIM1);
}

void pwm_set_superimpose_freq(const int freq)
{
  if (freq != 0)
    superimpose_cycle_ = PWM_HZ / freq;
  else
    superimpose_cycle_ = 0;
}

void pwm_set_superimpose_rate(const uint16_t rate)
{
  superimpose_compare_ = superimpose_cycle_ * rate / UINT16_MAX;
}

void pwm_disable_output()
{
  enabled_ = false;
}

void pwm_enable_output()
{
  enabled_ = true;
}

void pwm_cb()
{
  ++superimpose_cnt_;
  if (superimpose_cnt_ == superimpose_compare_)
  {
    LL_TIM_OC_SetCompareCH1(TIM1, oc_high_);
  }
  else if (superimpose_cnt_ >= superimpose_cycle_)
  {
    superimpose_cnt_ = 0;
    LL_TIM_OC_SetCompareCH1(TIM1, oc_low_);

    if (enabled_ == false || dir_ == PWM_DIR_IDLE)
    {
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
    }
    else if (dir_ == PWM_DIR_FWD)
    {
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    }
    else if (dir_ == PWM_DIR_REV)
    {
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    }
  }
}
/* USER CODE END 0 */

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = PWM_CYCLE;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /**TIM1 GPIO Configuration
    PA7     ------> TIM1_CH1N
    PA8     ------> TIM1_CH1
    */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
