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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const static uint32_t ReportIntervalDefault = 100;
const static uint32_t SwitchProcInterval = 10;
const static uint32_t EmoResetDuration = 5000;
const static char* NmeaPrefix = "$SSPP03";
const static uint32_t LedProcInterval = 125;
const static int LedPhaseMax = 8;
const static bool LedPatternStartup[] = {false, false, false, false, false, false, false, false};
const static bool LedPatternNormal[] = {false, false, false, false, true, true, true, true};
const static bool LedPatternEmo[] = {false, true, false, true, false, true, false, true};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ms = 0;
static uint32_t report_interval_ms_ = ReportIntervalDefault;
static uint32_t report_ms_prev_ = 0;
static uint32_t sw_proc_ms_prev_ = 0;
static uint32_t led_proc_ms_prev_ = 0;
static int led_phase_ = 0;
static STATE state = STATE_STARTUP;
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

int sw_mode()
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

// return size of packet
int nmea0183_add_suffix(char* buf)
{
  uint8_t sum = 0;
  int cnt = 0;
  if (buf[cnt++] != '$') return 0;
  do {
    sum ^= buf[cnt++];
    if (buf[cnt] == '\n' || buf[cnt] == '\0') return 0;
  } while (buf[cnt] != '*');
  buf[++cnt] = itoh((sum >> 4) & 0xF);
  buf[++cnt] = itoh((sum >> 0) & 0xF);
  buf[++cnt] = '\r';
  buf[++cnt] = '\n';
  buf[++cnt] = '\0';
  return cnt;
}

// return 1 : OK
//        0 : NG
int nmea0183_pop_word(char* dst, char** const src, const char delim)
{
  // copy characters until delimiter
  while (**src != '\0')  // EOL
  {
    if (**src == delim) // delimiter
    {
      (*src)++; // skip delimiter
      *dst = '\0';
      return 1;
    }
    *dst++ = *(*src)++;
  }
  return 0;
}

// return 1 : OK
//        0 : NG
int nmea0183_check_sum(const char* buf)
{
  uint8_t sum = 0;
  if (*buf++ != '$') return 0;
  do {
    sum ^= *buf++;
    if (*buf == '\n' || *buf == '\0') return 0;
  } while (*buf != '*');
  if (*(++buf) != itoh((sum >> 4) & 0xF))
    return 0;
  if (*(++buf) != itoh((sum >> 0) & 0xF))
    return 0;
  return 1;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char buf_rep[128];
  char buf_cmd[128];
  char buf_word[16];
  int rep_offset = 0;
  int vm_mv = 0;
  int cur_ma = 0;
  int mode = 0;
  int sw_mode_prev = 0;
  bool emo = false;
  bool emo_prev = false;
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
  /* USER CODE BEGIN 2 */
  LL_SYSTICK_EnableIT();
  usart_init();
  pwm_init();
  //adc_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  pwm_set_rate(0, PWM_DIR_IDLE);
  pwm_set_constant_light_rate(0);
  pwm_set_superimpose_amplitude(65535/5);
  pwm_set_superimpose_freq(20);

  led_pattern = LedPatternNormal;
  state = STATE_NORMAL;
  sw_proc_ms_prev_ = ms;
  report_ms_prev_ = ms;
  led_proc_ms_prev_ = ms;
  while (1)
  {
    vm_mv = adc_get_vm();
    cur_ma = adc_get_cur();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (emo != emo_prev)
    {
      // when change emo switch
      emo_change_ms = ms;
      state_emo_changed = state;
    }
    emo_prev = emo;

    if (emo
        && state == STATE_EMO
        && state_emo_changed == STATE_EMO
        && ms - emo_change_ms >= EmoResetDuration)
    {
      // reset emo state
      led_pattern = LedPatternNormal;
      state = STATE_NORMAL;
      pwm_set_rate(0, PWM_DIR_IDLE);
      pwm_enable_output();
    }
    else if (emo && state_emo_changed != STATE_EMO)
    {
      led_pattern = LedPatternEmo;
      state = STATE_EMO;
      pwm_disable_output();
    }

    while(usart2_gets(buf_cmd))
    {
      long cmd_id;
      char* parse_point;
      char* work_point;

      // skip until leader character
      if (*buf_cmd != '$')
        continue;

      // test check sum
      if (nmea0183_check_sum(buf_cmd) == 0)
        continue;

      // check prefix
      parse_point = buf_cmd;
      if (nmea0183_pop_word(buf_word, &parse_point, ',') == 0)
        continue;
      if (strncmp(buf_word, NmeaPrefix, 7))
        continue;

      // read device id
      if (nmea0183_pop_word(buf_word, &parse_point, ',') == 0)
        continue;
      work_point = buf_word;
      if(xatoi(&work_point, &cmd_id) == 0)
        continue;
      if (cmd_id != 1)
      {
        // TODO
        // transfer to follower device
        continue;
      }

      // parse sub command
      if (nmea0183_pop_word(buf_word, &parse_point, ',') == 0)
        continue;
      if (strncmp(buf_word, "CMDSLT", 6) == 0)
      {
        // Set thlottle value command
        // $SSPP03,id,CMDSLT,d,t*cc
        // d : direction
        //      1 : fwd
        //      0 : idle
        //      -1 : rev
        // t : thlottle
        //      0 to 65535
        long dir = 0;
        long thlottle = 0;
        if (nmea0183_pop_word(buf_word, &parse_point, ',') == 0)
          continue;
        work_point = buf_word;
        if(xatoi(&work_point, &dir) == 0)
          continue;
        if (nmea0183_pop_word(buf_word, &parse_point, '*') == 0)
          continue;
        work_point = buf_word;
        if(xatoi(&work_point, &thlottle) == 0)
          continue;
        pwm_set_rate(thlottle, dir);
      }
      else if (strncmp(buf_word, "CMDCLT", 6) == 0)
      {
        // Set constant light value command
        // $SSPP03,id,CMDCLT,r*cc
        long rate = 0;
        if (nmea0183_pop_word(buf_word, &parse_point, '*') == 0)
          continue;
        work_point = buf_word;
        if(xatoi(&work_point, &rate) == 0)
          continue;
        pwm_set_constant_light_rate(rate);
      }
      else if (strncmp(buf_word, "CMDSIA", 6) == 0)
      {
        // Set superimpose amplitude value command
        // $SSPP03,id,CMDSIA,a*cc
        long amp = 0;
        if (nmea0183_pop_word(buf_word, &parse_point, '*') == 0)
          continue;
        work_point = buf_word;
        if(xatoi(&work_point, &amp) == 0)
          continue;
        pwm_set_superimpose_amplitude(amp);
      }
      else if (strncmp(buf_word, "CMDRST", 6) == 0)
      {
        // Reset emergency stop state
        // $SSPP03,id,CMDRST*cc
        if (state == STATE_EMO)
        {
          pwm_set_rate(0, PWM_DIR_IDLE);
          pwm_enable_output();
          led_pattern = LedPatternNormal;
          state = STATE_NORMAL;
        }
      }
      else if (strncmp(buf_word, "CMDEMO", 6) == 0)
      {
        // Enter to emergency stop state
        pwm_disable_output();
        led_pattern = LedPatternEmo;
        state = STATE_EMO;
      }
    }

    if (ms - sw_proc_ms_prev_ >= SwitchProcInterval)
    {
      sw_proc_ms_prev_ += SwitchProcInterval;
      bool s = sw_emo_is_pressed();
      if (s == sw_emo_prev) emo = s;
      sw_emo_prev = s;

      int m = sw_mode();
      if (m == sw_mode_prev) mode = m;
      sw_mode_prev = m;
    }

    if (ms - report_ms_prev_ >= report_interval_ms_)
    {
      report_ms_prev_ += report_interval_ms_;
      rep_offset = 0;
      xsprintf(buf_rep, "%s,1,REPPWR,%d,%d*", NmeaPrefix, vm_mv, cur_ma);
      rep_offset += nmea0183_add_suffix(buf_rep);
      xsprintf(buf_rep+rep_offset, "%s,1,REPSTA,%d,%d,%d*", NmeaPrefix, emo ? 1 : 0, mode, state);
      rep_offset += nmea0183_add_suffix(buf_rep+rep_offset);
      usart2_puts(buf_rep);
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
