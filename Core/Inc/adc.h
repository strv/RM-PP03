/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define ADC_SUM_Q   (4)
#define ADC_SUM_NUM (1 << ADC_SUM_Q)
#define ADC_BUF_LEN (ADC_CH_NUM * ADC_SUM_NUM)
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef enum
{
  ADC_CH_VM = 0,
  ADC_CH_CUR,
  ADC_CH_AUX0,
  ADC_CH_AUX1,
  ADC_CH_VREF,
  ADC_CH_NUM
}ADC_CH;
void adc_init();
void adc_dma_cb();
int adc_get_vm();
int adc_get_cur();
int adc_get_vm_peak();
int adc_get_cur_peak();
void adc_trigger();
void adc_set_pwm_rate(const uint16_t rate);
void adc_lpf_proc();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

