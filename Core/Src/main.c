/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "buzzer.h"
#include "boot_splash.h"
#include "tft_ui.h"
#include "st7789.h"
#include "control.h"
#include "power_mgr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_ms_tick = 0;
static volatile uint8_t g_power_fail_pending = 0U;
static volatile uint8_t g_power_recovery_pending = 0U;

#define SYSTEM_IWDG_RELOAD_VALUE 1400U
#define SYSTEM_IWDG_PRESCALER_DIV128 5U
#define SYSTEM_IWDG_KEY_UNLOCK 0x5555U
#define SYSTEM_IWDG_KEY_REFRESH 0xAAAAU
#define SYSTEM_IWDG_KEY_START 0xCCCCU
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void System_WatchdogInit(void);

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  {
    PWR_PVDTypeDef pvd_config = {0};
    pvd_config.PVDLevel = PWR_PVDLEVEL_7;
    pvd_config.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
    HAL_PWR_ConfigPVD(&pvd_config);
    HAL_PWR_EnablePVD();
    HAL_NVIC_SetPriority(PVD_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(PVD_IRQn);
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET)
    {
      g_power_fail_pending = 1U;
    }
  }
  HAL_TIM_Base_Start_IT(&htim4);
  Buzzer_Init();
  PowerMgr_Init();
  Encoder_Init();
  TftUi_Init();
  Control_Init();
  TftUi_SetWorkState(TFT_UI_STATE_READY);
  ST7789_Init();
  BootSplash_Show();
  System_WatchdogInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    EncoderEvent_t encoder_event = ENCODER_EVENT_NONE;

    if (g_power_fail_pending != 0U)
    {
      /* 1. 立即关闭所有高功耗外设，为Flash保存预留电能 */
      Control_PowerFailShutdown(); 
      Buzzer_Off();
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

      /* 2. 执行关键数据保存 */
      (void)TftUi_SavePendingCount();

      HAL_TIM_Base_Stop_IT(&htim4);
      g_power_fail_pending = 0U;

      /* 保持 PVD 中断开启，电压恢复后复位并重新启动设备。 */
      while (1)
      {
        IWDG->KR = SYSTEM_IWDG_KEY_REFRESH;
        if ((g_power_recovery_pending != 0U) &&
            (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == RESET))
        {
          HAL_Delay(100U);
          if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) == RESET)
          {
            NVIC_SystemReset();
          }
          g_power_recovery_pending = 0U;
        }
        __WFI();
      }
    }

    PowerMgr_Task();
    Control_HandleTouch();
    Control_Task();
    Encoder_Task();
    encoder_event = Encoder_GetEvent();
    if (encoder_event != ENCODER_EVENT_NONE)
    {
      TftUi_ProcessEncoderEvent(encoder_event);
    }
    Buzzer_Task();
    PowerMgr_ChargeTask();
    if ((encoder_event == ENCODER_EVENT_CLICK) ||
        (encoder_event == ENCODER_EVENT_LONG_PRESS))
    {
      Buzzer_Beep(10U);
    }
    TftUi_Task();
    IWDG->KR = SYSTEM_IWDG_KEY_REFRESH;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void System_WatchdogInit(void)
{
  uint32_t update_timeout = 0U;

  IWDG->KR = SYSTEM_IWDG_KEY_UNLOCK;
  IWDG->PR = SYSTEM_IWDG_PRESCALER_DIV128;
  IWDG->RLR = SYSTEM_IWDG_RELOAD_VALUE;
  while ((IWDG->SR != 0U) && (update_timeout < 100000U))
  {
    update_timeout++;
  }
  IWDG->KR = SYSTEM_IWDG_KEY_REFRESH;
  IWDG->KR = SYSTEM_IWDG_KEY_START;
}

void System_FaultShutdown(void)
{
  GPIOB->BRR = CH_EN_Pin;
  GPIOA->BRR = C1_Pin | C2_Pin | C3_Pin | C4_Pin | BUZZER_Pin;
  TIM1->CCR1 = 0U;
  TIM2->CCR4 = 0U;
  __DSB();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 使用 TIM4 周期中断累计系统毫秒计数。 */
  if (htim->Instance == TIM4)
  {
    g_ms_tick++;
  }
}

void PVD_IRQHandler(void)
{
  /* 将电源电压检测中断交由 HAL 处理。 */
  HAL_PWR_PVD_IRQHandler();
}

void HAL_PWR_PVDCallback(void)
{
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET)
  {
    System_FaultShutdown();
    g_power_fail_pending = 1U;
    g_power_recovery_pending = 0U;
  }
  else
  {
    g_power_recovery_pending = 1U;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  System_FaultShutdown();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
