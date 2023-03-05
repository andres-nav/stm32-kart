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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned char is_trigger_on = 0;
unsigned short time_init = 0;
unsigned char do_calculate_time = 0;
unsigned short delay = 0;
float distance = 0;

void TIM3_IRQHandler(void) {
  if ((TIM3->SR & 0x0002) != 0) {  // If the comparison is successful, then the IRQ is
  // launched and this ISR is executed. This line check
  // which event launched the ISR
    if (is_trigger_on == 1) {
      GPIOC->BSRR |= (1 << 2) << 16;
      is_trigger_on = 0;
      TIM3->CR1 &= ~(0x0001);   // CxEN = 0 -> Stop counter

      TIM2->CR1 |= 0x0001;   // CEN = 1 -> Start counter
      TIM2->EGR |= 0x0001;   // UG = 1 -> Generate update event
      TIM2->SR = 0;
      time_init = TIM2->CCR1;

    } else if (is_trigger_on == 0) {
      GPIOC->BSRR |= (1 << 2);
      is_trigger_on = 1;
    }
    TIM3->SR = 0;
  }
}

void TIM2_IRQHandler(void) {
  if ((TIM2->SR & 0x0002)!=0) {            // Check that the event is the TIC selected
    if ((GPIOA->IDR & (1 << 5)) != 0) {
      time_init = TIM2->CCR1;
    } else {
      delay = TIM2->CCR1 - time_init;
      if (delay<0) delay += 0x0FFFF;      // Handle counter overflows
      distance = delay * 0.00068;
      time_init = 0;
      do_calculate_time = 1;


      TIM2->CR1 &= ~(0x0001);   // CEN = 0 -> Stop counter

      TIM3->CR1 |= 0x0001;   // CEN = 1 -> Start counter
      TIM3->EGR |= 0x0001;   // UG = 1 -> Generate update event
      TIM3->SR = 0;
    }

    TIM2->SR = 0;                    // Clear flags
  }
}

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
  MX_ADC_Init();
  MX_TS_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Buzzer
  GPIOA->MODER &= ~(1 << (2*1 + 1)); // pa1
  GPIOA->MODER |= (1 << (2*1));

  GPIOA->OTYPER &= ~(1 << 1);

  GPIOA->OSPEEDR &= ~(1 << (2*1 +1));
  GPIOA->OSPEEDR &= ~(1 << (2*1));

  // Trigger
  GPIOC->MODER &= ~(1 << (2*2 + 1)); // pc2
  GPIOC->MODER |= (1 << (2*2));

  GPIOC->OTYPER &= ~(1 << 2);

  GPIOC->OSPEEDR &= ~(1 << (2*2 +1));
  GPIOC->OSPEEDR &= ~(1 << (2*2));


  // Internal clock selection: CR1, CR2, SMRC
  TIM3->CR1 = 0x0000;                    // ARPE = 0; CEN = 0; Counter OFF
  TIM3->CR2 = 0x0000;                    // Always 0 in this course
  TIM3->SMCR = 0x0000;                   // Always 0 in this course
  // Counter setting: PSC, CNT, ARR and CCRx

  TIM3->PSC = 1 - 1;      // Pre-scaler=32000 -> f_cnt = 1000 steps/second
  TIM3->CNT = 0;          // Initialize counter to 0
  TIM3->ARR = 0xFFFF;     // Maximum excursion of CNT
  TIM3->CCR1 = 4;
  // Select, or not, IRQ: DIER
  TIM3->DIER |= (1<<1);   // IRQ when CCR1 is reached -> CCyIE = 1
  // Output mode
  TIM3->CCMR1 &= ~(0x00FF); // Clear all channel 1 information
  TIM3->CCMR1 |= 0x0030;    // CC1S = 0   (TOC, PWM)
  // OC1M = 011 (Toggle)
  // OC1PE = 0  (without preload)
  TIM3->CCER &= ~(0x000F);

  TIM3->CCER |= 0x0001;  // CC1P = 0   (always)
  // CC1E = 1   (hardware output activated)
  // Counter enabling
  // Enabling TIM3_IRQ at NVIC (position 30).
  NVIC->ISER[0] |= (1 << 29);


  //----------------------------------
  GPIOA->MODER |= 1 << (2*5 +1);    // MODER = 10 (AF) for PA5
  GPIOA->MODER &= ~(1 << (2*5));
  GPIOA->AFR[0] |= (1 << (4*5));          // AFR for PA5 as AF1 (TIM2)
  GPIOA->AFR[0] &= ~(0x0E << (4*5));
  // Internal clock selection: CR1, CR2, SMRC
  TIM2->CR1 = 0x0000;       // ARPE = 0 -> No PWM, but TIC; CEN = 0; Counter OFF
  TIM2->CR2 = 0x0000;       // CCyIE = 0 -> No IRQ
  TIM2->SMCR = 0x0000;      // Always 0 in this course
  // Counter setting: PSC, CNT, ARR y CCRx
  TIM2->PSC = 2;      // Pre-scaler=1 -> 2.5 microseconds/step
  TIM2->CNT = 0;          // Counter initialized to 0
  TIM2->ARR = 0xFFFF;     // As recommended when no PWM
  // IRQ or not IRQ selection: DIER
  TIM2->DIER = 0x0002;    // IRQ generate for TIC
  // External pin behaviour
  TIM2->CCMR1 = 0x0001;  // CCyS = 1 (TIC); OCyM = 000 y OCyPE = 0 (always in TIC)
  TIM2->CCER = 0x0001;   // CCyNP:CCyP = 00 (rising and falling edge active)
  TIM2->CCER |= (1 << 1);
  TIM2->CCER |= (1 << 3);

  NVIC->ISER[0] |= (1 << 28);

  // Turn on timers
  TIM3->CR1 |= 0x0001;   // CEN = 1 -> Start counter
  TIM3->EGR |= 0x0001;   // UG = 1 -> Generate update event
  TIM3->SR = 0;          // Counter flags cleared (for all channels)


  // CCyE = 1        (capture enabled for TIC)
  // Habilitación de contador
  TIM2->SR = 0;          // Clear flags

  unsigned char count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if (do_calculate_time == 1) {
      do_calculate_time = 0;
      if (distance <= 10) {
        count++;
        if (count > 10) {
          GPIOA->BSRR |= (1 << 1);
        }
      } else {
        count = 0;
        GPIOA->BSRR |= (1 << 1) << 16;
      }
    }

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG14_Pin SEG15_Pin SEG16_Pin SEG17_Pin
                           SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin
                           SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG14_Pin|SEG15_Pin|SEG16_Pin|SEG17_Pin
                          |SEG18_Pin|SEG19_Pin|SEG20_Pin|SEG21_Pin
                          |SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin COM0_Pin
                           COM1_Pin COM2_Pin SEG12_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|COM0_Pin
                          |COM1_Pin|COM2_Pin|SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG6_Pin SEG7_Pin SEG8_Pin SEG9_Pin
                           SEG10_Pin SEG11_Pin SEG3_Pin SEG4_Pin
                           SEG5_Pin SEG13_Pin COM3_Pin */
  GPIO_InitStruct.Pin = SEG6_Pin|SEG7_Pin|SEG8_Pin|SEG9_Pin
                          |SEG10_Pin|SEG11_Pin|SEG3_Pin|SEG4_Pin
                          |SEG5_Pin|SEG13_Pin|COM3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
