/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

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

/* USER CODE END Private defines */

void MX_TIM1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef enum {
  PWM_DIR_REV = -1,
  PWM_DIR_IDLE = 0,
  PWM_DIR_FWD = 1
}PWM_DIR;

void pwm_init(void);
void pwm_set_constant_light_rate(const uint16_t rate_cl);
void pwm_set_supoerimpose_amplitude(const uint16_t rate_superimpose);
void pwm_set_rate(const uint16_t rate, const PWM_DIR dir);
void pwm_set_superimpose_freq(const int freq);
void pwm_set_superimpose_rate(const uint16_t rate);
void pwm_disable_output();
void pwm_enable_output();
void pwm_cb();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

