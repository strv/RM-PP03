/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STATE_STARTUP,
  STATE_NORMAL,
  STATE_EMO,
}STATE;

typedef enum
{
  MB_EC_ILLEGAL_FUNCTION = 0x01,
  MB_EC_ILLEGAL_DATA_ADDRESS,
  MB_EC_ILLEGAL_DATA_VALUE,
  MB_EC_SERVER_DEVICFEFAILURE,
  MB_EC_ACKNOWLEDGE,
  MB_EC_SERVER_DEVICE_BUSY,
  MB_EC_MEMORY_PARITY_ERROR = 0x08,
  MB_EC_GATEWAY_PATH_UNAVAILABLE = 0x0A,
  MB_EC_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND,
}MB_EXCEPTION_CODE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const static uint32_t ModbusTimeout = 1000;
const static uint32_t SwitchProcInterval = 10;
const static uint32_t EmoResetDuration = 5000;
const static uint32_t LedProcInterval = 125;
const static int LedPhaseMax = 8;
const static bool LedPatternStartup[] = {false, false, false, false, false, false, false, false};
const static bool LedPatternDisabled[] = {false, false, false, false, false, true, false, true};
const static bool LedPatternEnabled[] = {false, false, false, false, true, true, true, true};
const static bool LedPatternEmo[] = {false, true, false, true, false, true, false, true};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ms = 0;
static uint32_t last_mb_ms_ = 0;
static uint32_t sw_proc_ms_prev_ = 0;
static uint32_t led_proc_ms_prev_ = 0;
static int led_phase_ = 0;
static STATE state = STATE_STARTUP;
static bool adc_update_ = false;

const int MbSlaveAddress = 1;
/*
Modbus Coils
Addr  : Assign
1     : Output
2     : Emergency mode
*/
typedef enum
{
  MB_COILS_OUTPUT = 0,
  MB_COILS_EMERGENCY_MODE,
  MB_COILS_NUM
} MB_COILS;
static bool mb_coils_[MB_COILS_NUM] = {false};
/*
Modbus Discrete inputs
Addr  : Assign
1     : Emergency switch
2     : Mode switch
*/
typedef enum
{
  MB_DI_SW_EMO = 0,
  MB_DI_SW_MODE,
  MB_DI_NUM
} MB_DISCRETE_INPUTS;
static bool mb_discrete_inputs_[MB_DI_NUM] = {false};
/*
Modbus Holding Registers
Addr  : Assign
40001 : Output rate
          bit 0-14  : rate 0 to 32767
          bit 15    : direction 0 : Foward / 1 : Reverse
40002 : Constant light rate 0 to 65535
40003 : Superimpose amplitude rate 0 to 65535. default 65535/5
40004 : Superimpose frequency 0 to 1000 is beter. default 50
*/
typedef enum
{
  MB_HR_OUTPUT_RATE = 0,
  MB_HR_CL_RATE,
  MB_HR_SI_RATE,
  MB_HR_SI_FREQ,
  MB_HR_NUM
} MB_HOLDING_REGISTERS;
static uint16_t mb_holding_regs_[MB_HR_NUM] = {
  0,
  0,
  65535/5,
  50
};
/*
Modbus Input Registers
Addr  : Assign
30001 : Average supply voltage in mV
30002 : Peak supply voltage in mV
30003 : Average output current in mA
30004 : Peak output current in mA
*/
typedef enum
{
  MB_IR_VOLT_AVE = 0,
  MB_IR_VOLT_PEAK,
  MB_IR_CUR_AVE,
  MB_IR_CUR_PEAK,
  MB_IR_NUM
} MB_INPUT_REGISTERS;
static uint16_t mb_input_regs_[MB_IR_NUM] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char itoh(const int i)
{
  if (i > 0xF) return 0;
  if (i > 9) return i - 10 + 'A';
  return i + '0';
}

bool sw_emo_is_pressed()
{
  return LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_14) > 0 ? false : true;
}

int sw_mode_state()
{
  return LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_15) > 0 ? 1 : 0;
}

void led_on()
{
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

void led_off()
{
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adc_update_reserve()
{
  adc_update_ = true;
}

void init_boot_config()
{
  if (FLASH->OPTR & (FLASH_OPTR_nBOOT_SEL_Msk))
  {
    // Unlock FLASH programing
    WRITE_REG(FLASH->KEYR, 0x45670123);
    WRITE_REG(FLASH->KEYR, 0xCDEF89AB);

    // Unlock FLASH option programing
    WRITE_REG(FLASH->OPTKEYR, 0x08192A3B);
    WRITE_REG(FLASH->OPTKEYR, 0x4C5D6E7F);

    // Enable BOOT0 pin
    CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT_SEL);

    // Check no FLASH operation
    while (FLASH->SR & FLASH_SR_BSY1_Msk);

    // FLASH program start
    SET_BIT(FLASH->CR, FLASH_CR_OPTSTRT);

    // Wait program done
    while (FLASH->SR & FLASH_SR_BSY1_Msk);

    // Load to register
    SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH);
  }
}

void update_pwm_settings()
{
  PWM_DIR dir = mb_holding_regs_[MB_HR_OUTPUT_RATE] & 0x8000 ? PWM_DIR_REV : PWM_DIR_FWD;
  pwm_set_rate((mb_holding_regs_[MB_HR_OUTPUT_RATE] & 0x7FFFF) * 2, dir);
  pwm_set_constant_light_rate(mb_holding_regs_[MB_HR_CL_RATE]);
  pwm_set_superimpose_amplitude(mb_holding_regs_[MB_HR_SI_RATE]);
  pwm_set_superimpose_freq(mb_holding_regs_[MB_HR_SI_FREQ]);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t mb_rx_frame[256];
  uint8_t mb_tx_frame[256];
  int mb_length = 0;
  int sw_mode = 0;
  int sw_mode_prev = 0;
  bool sw_emo = false;
  bool emo_prev = false;
  bool mb_emo_prev = false;
  uint32_t emo_change_ms = 0;
  STATE state_emo_changed = state;
  bool sw_emo_prev = false;
  const bool* led_pattern = LedPatternStartup;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */
  init_boot_config();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  LL_SYSTICK_EnableIT();
  usart_init();
  pwm_init();
  adc_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  update_pwm_settings();

  state = STATE_NORMAL;
  sw_proc_ms_prev_ = ms;
  led_proc_ms_prev_ = ms;
  while (1)
  {
    if (adc_update_)
    {
      adc_update_ = false;
      adc_lpf_proc();
      mb_input_regs_[MB_IR_VOLT_AVE] = adc_get_vm();
      mb_input_regs_[MB_IR_VOLT_PEAK] = adc_get_vm_peak();
      mb_input_regs_[MB_IR_CUR_AVE] = adc_get_cur();
      mb_input_regs_[MB_IR_CUR_PEAK] = adc_get_cur_peak();
    }
    mb_discrete_inputs_[MB_DI_SW_EMO]  = sw_emo;
    mb_discrete_inputs_[MB_DI_SW_MODE] = sw_mode;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    mb_length = mb_pop_frame(mb_rx_frame);
    if (mb_length != 0)
    {
      const uint8_t mb_slv_addr = mb_rx_frame[0];
      const uint8_t mb_func_code = mb_rx_frame[1];
      const uint8_t* mb_pdata = &mb_rx_frame[2];
      if (mb_slv_addr != 1)
      {

      }
      else
      {
        int len = 0;
        mb_tx_frame[len++] = 0x01;
        switch (mb_func_code)
        {
          // 01 (0x01) Read Coils
          case 0x01:
          {
            const uint16_t starting_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t quantity_of_coils = mb_pdata[2] << 8 | mb_pdata[3];
            if (starting_addr > MB_COILS_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else if (quantity_of_coils == 0 || quantity_of_coils + starting_addr > MB_COILS_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_VALUE;
            }
            else
            {
              mb_tx_frame[len++] = mb_func_code;
              const int byte_count = (quantity_of_coils + 7) / 8;
              mb_tx_frame[len++] = byte_count;
              for (int i = 0; i < byte_count; ++i)
              {
                for (int j = 0; j < 8; ++j)
                {
                  const int idx = starting_addr + i*8 + j;
                  if (idx == MB_COILS_NUM)
                    break;
                  if (mb_coils_[idx])
                    mb_tx_frame[len + i] |= (1 << j);
                  else
                    mb_tx_frame[len + i] &= ~(1 << j);
                }
              }
              len += byte_count;
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          // 02 (0x02) Read Discrete Inputs
          case 0x02:
          {
            const uint16_t starting_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t quantity_of_inputs = mb_pdata[2] << 8 | mb_pdata[3];
            if (starting_addr > MB_DI_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else if (quantity_of_inputs == 0 || quantity_of_inputs + starting_addr > MB_DI_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_VALUE;
            }
            else
            {
              mb_tx_frame[len++] = mb_func_code;
              const int byte_count = (quantity_of_inputs + 7) / 8;
              mb_tx_frame[len++] = byte_count;
              for (int i = 0; i < byte_count; ++i)
              {
                for (int j = 0; j < 8; ++j)
                {
                  const int idx = starting_addr + i*8 + j;
                  if (idx == MB_DI_NUM)
                    break;
                  if (mb_discrete_inputs_[idx])
                    mb_tx_frame[len + i] |= (1 << j);
                  else
                    mb_tx_frame[len + i] &= ~(1 << j);
                }
              }
              len += byte_count;
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          // 03 (0x03) Read Holding Registers
          case 0x03:
          {
            const uint16_t starting_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t quantity_of_reg = mb_pdata[2] << 8 | mb_pdata[3];
            if (starting_addr > MB_HR_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else if (quantity_of_reg == 0 || quantity_of_reg + starting_addr > MB_HR_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_VALUE;
            }
            else
            {
              mb_tx_frame[len++] = mb_func_code;
              mb_tx_frame[len++] = quantity_of_reg * 2;
              for (int i = 0; i < quantity_of_reg; ++i)
              {
                mb_tx_frame[len++] = mb_holding_regs_[starting_addr + i] >> 8;
                mb_tx_frame[len++] = mb_holding_regs_[starting_addr + i] & 0xFF;
              }
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          // 04 (0x04) Read Input Registers
          case 0x04:
          {
            const uint16_t starting_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t quantity_of_reg = mb_pdata[2] << 8 | mb_pdata[3];
            if (starting_addr > MB_IR_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else if (quantity_of_reg == 0 || quantity_of_reg + starting_addr > MB_IR_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_VALUE;
            }
            else
            {
              mb_tx_frame[len++] = mb_func_code;
              mb_tx_frame[len++] = quantity_of_reg * 2;
              for (int i = 0; i < quantity_of_reg; ++i)
              {
                mb_tx_frame[len++] = mb_input_regs_[starting_addr + i] >> 8;
                mb_tx_frame[len++] = mb_input_regs_[starting_addr + i] & 0xFF;
              }
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          // 05 (0x05) Write Single Coil
          case 0x05:
          {
            const uint16_t output_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t output_value = mb_pdata[2] << 8 | mb_pdata[3];
            if (output_addr > MB_COILS_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else if (output_value == 0x0000 || output_value == 0xFF00)
            {
              memcpy(mb_tx_frame, mb_rx_frame, mb_length);
              len = mb_length;
              mb_coils_[output_addr] = output_value != 0 ? true : false;
            }
            else
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_VALUE;
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          // 06 (0x06) Write Single Register
          case 0x06:
          {
            const uint16_t reg_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t reg_value = mb_pdata[2] << 8 | mb_pdata[3];
            if (reg_addr > MB_HR_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else
            {
              memcpy(mb_tx_frame, mb_rx_frame, mb_length);
              len = mb_length;

              mb_holding_regs_[reg_addr] = reg_value;
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          // 15 (0x0F) Write Multiple Coils
          case 0x0F:
          {
            const uint16_t starting_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t quantity_of_coils = mb_pdata[2] << 8 | mb_pdata[3];
            const uint8_t byte_count = mb_pdata[4];
            if (quantity_of_coils == 0
                || (quantity_of_coils + 7) / 8 != byte_count
                || byte_count != mb_length - (2+5+2))
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_VALUE;
            }
            else if (quantity_of_coils + starting_addr > MB_COILS_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else
            {
              memcpy(mb_tx_frame, mb_rx_frame, mb_length);
              len = mb_length;

              for (int i = 0; i < quantity_of_coils; ++i)
              {
                if (mb_pdata[5 + i / 8] & (1 << (i%8)))
                  mb_coils_[i + starting_addr] = true;
                else
                  mb_coils_[i + starting_addr] = false;
              }
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          // 16 (0x10) Write Multiple registers
          case 0x10:
          {
            const uint16_t starting_addr = mb_pdata[0] << 8 | mb_pdata[1];
            const uint16_t quantity_of_regs = mb_pdata[2] << 8 | mb_pdata[3];
            const uint8_t byte_count = mb_pdata[4];
            if (quantity_of_regs == 0
                || quantity_of_regs * 2 != byte_count
                || byte_count != mb_length - (2+5+2))
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_VALUE;
            }
            else if (quantity_of_regs + starting_addr > MB_HR_NUM)
            {
              mb_tx_frame[len++] = mb_func_code + 0x80;
              mb_tx_frame[len++] = MB_EC_ILLEGAL_DATA_ADDRESS;
            }
            else
            {
              memcpy(mb_tx_frame, mb_rx_frame, mb_length);
              len = mb_length;
              for (int i = 0; i < quantity_of_regs; ++i)
              {
                mb_holding_regs_[starting_addr + i] = mb_pdata[i * 2 + 5] << 8 | mb_pdata[i * 2 + 6];
              }
            }
            mb_push_frame(mb_tx_frame, len);
          }
          break;

          default:
          {
            mb_tx_frame[len++] = mb_func_code + 0x80;
            mb_tx_frame[len++] = MB_EC_ILLEGAL_FUNCTION;
            mb_push_frame(mb_tx_frame, len);
          }
          break;
        }
      }
      last_mb_ms_ = ms;
    }

    if (mb_coils_[MB_COILS_OUTPUT])
    {
      if (sw_emo != emo_prev)
      {
        // when change emo switch
        emo_change_ms = ms;
        state_emo_changed = state;
      }
      emo_prev = sw_emo;

      if ( (mb_coils_[MB_COILS_EMERGENCY_MODE] && !mb_emo_prev)
        || (sw_emo && state_emo_changed != STATE_EMO)
        || (ms - last_mb_ms_ > ModbusTimeout))
      {
        // enter to emo state
        led_pattern = LedPatternEmo;
        state = STATE_EMO;
        mb_coils_[MB_COILS_EMERGENCY_MODE] = true;
        pwm_disable_output();
      }
      else if ( (!mb_coils_[MB_COILS_EMERGENCY_MODE] && mb_emo_prev)
          || (sw_emo
          && state == STATE_EMO
          && state_emo_changed == STATE_EMO
          && ms - emo_change_ms >= EmoResetDuration))
      {
        // reset emo state
        led_pattern = LedPatternEnabled;
        state = STATE_NORMAL;
        mb_holding_regs_[MB_HR_OUTPUT_RATE] = 0;
        mb_coils_[MB_COILS_EMERGENCY_MODE] = false;
        update_pwm_settings();
        pwm_enable_output();
      }
      else if (state == STATE_NORMAL)
      {
        led_pattern = LedPatternEnabled;
        update_pwm_settings();
        pwm_enable_output();
      }
      mb_emo_prev = mb_coils_[MB_COILS_EMERGENCY_MODE];
    }
    else
    {
      led_pattern = LedPatternDisabled;
      pwm_disable_output();
    }

    if (ms - sw_proc_ms_prev_ >= SwitchProcInterval)
    {
      sw_proc_ms_prev_ += SwitchProcInterval;
      bool s = sw_emo_is_pressed();
      if (s == sw_emo_prev) sw_emo = s;
      sw_emo_prev = s;

      int m = sw_mode_state();
      if (m == sw_mode_prev) sw_mode = m;
      sw_mode_prev = m;
    }

    if (ms - led_proc_ms_prev_ >= LedProcInterval)
    {
      led_proc_ms_prev_ += LedProcInterval;
      if (led_pattern[led_phase_++])
        led_on();
      else
        led_off();
      if (led_phase_ == LedPhaseMax)
        led_phase_ = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
