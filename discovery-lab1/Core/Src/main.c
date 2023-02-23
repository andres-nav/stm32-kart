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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Waits a specific number of seconds approx.
 */
void espera(char seconds) {
  for (int i=0; i<(seconds*2000000); i++);
}

/*
 * Initializes the pins of the motors as digital output with default.
 * 	type and speed
 */
void initMotorPin(MotorPin *motor_pin, GPIO_TypeDef *gpio, char pin) {
  motor_pin->gpio = gpio;
  motor_pin->pin = pin;

  gpio->MODER &= ~(1 << (pin*2 + 1));
  gpio->MODER |= (1 << (pin*2));

  gpio->OTYPER &= ~(1 << pin);

  gpio->OSPEEDR &= ~(1 << (pin*2 +1));
  gpio->OSPEEDR &= ~(1 << (pin*2));
}

/*
 * Initializes the motor with the corresponding pins.
 */
void initMotor(Motor *motor, MotorPin *pin_1, MotorPin *pin_2) {
  motor->pin_1 = *pin_1;
  motor->pin_2 = *pin_2;
}

/*
 * Initializes the robot with the motors and stops it.
 */
void initRobot(Robot *robot, Motor *motor_right, Motor *motor_left) {
  robot->motor_right = *motor_right;
  robot->motor_left = *motor_left;

  updateStatusRobot(robot, ROBOT_STOPPED);
}

/*
 * Updates the status of the motor pin and calls to implement the status.
 */
void updateStatusMotorPin(MotorPin *motor_pin, StatusMotorPin status) {
  motor_pin->status = status;
  updateMotorPin(motor_pin);
}

/*
 * Updates the status of the motor and calls to implement the status.
 */
void updateStatusMotor(Motor *motor, StatusMotor status) {
  motor->status = status;
  updateMotor(motor);
}

/*
 * Updates the status of the motor and calls to implement the status.
 */
void updateStatusRobot(Robot *robot, StatusRobot status) {
  robot->status = status;
  updateRobot(robot);
}

/*
 * Updates the motor pin depending of its status
 * 	MOTOR_PIN_UP: sets the BSRR register to set the pin to 1
 * 	MOTOR_PIN_DOWN: sets the BSRR register to set the pin to 0
 */
void updateMotorPin(MotorPin *motor_pin) {
  switch (motor_pin->status) {
    case MOTOR_PIN_UP:
      motor_pin->gpio->BSRR |= (1 << motor_pin->pin);

      break;
    case MOTOR_PIN_DOWN:
      motor_pin->gpio->BSRR |= (1 << motor_pin->pin) << 16;

      break;
  }

}

/*
 * Updates the motor depending of its status
 * 	MOTOR_STOPPED: sets the motor to stop
 * 	MOTOR_FORWARD: sets the motor to forward with respect to the whole robot
 * 	MOTOR_BACKWARD: sets the motor to backward with respect to the whole robot
 *
 * 	This is important as one motor cables are swapped to correct that it is flip. (hardware)
 */
void updateMotor(Motor *motor) {
  StatusMotorPin status_motor_pin_1, status_motor_pin_2;

  switch (motor->status) {
    case MOTOR_STOPPED:
      status_motor_pin_1 = MOTOR_PIN_UP;
      status_motor_pin_2 = MOTOR_PIN_UP;

      break;
    case MOTOR_FORWARD:
      status_motor_pin_1 = MOTOR_PIN_UP;
      status_motor_pin_2 = MOTOR_PIN_DOWN;

      break;
    case MOTOR_BACKWARD:
      status_motor_pin_1 = MOTOR_PIN_DOWN;
      status_motor_pin_2 = MOTOR_PIN_UP;

      break;
  }

  updateStatusMotorPin(&(motor->pin_1), status_motor_pin_1);
  updateStatusMotorPin(&(motor->pin_2), status_motor_pin_2);

}

/*
 * Updates the robot to its according status
 * 	All the movements are with respect to the whole robot.
 */
void updateRobot(Robot *robot) {
  StatusMotor status_motor_right, status_motor_left;

  switch (robot->status) {
      case ROBOT_STOPPED:
        status_motor_right = MOTOR_STOPPED;
        status_motor_left = MOTOR_STOPPED;

        break;
      case ROBOT_FORWARD:
        status_motor_right = MOTOR_FORWARD;
        status_motor_left = MOTOR_FORWARD;

        break;
      case ROBOT_BACKWARD:
        status_motor_right = MOTOR_BACKWARD;
        status_motor_left = MOTOR_BACKWARD;

        break;
      case ROBOT_RIGHT:
        status_motor_right = MOTOR_BACKWARD;
        status_motor_left = MOTOR_FORWARD;

        break;
      case ROBOT_LEFT:
        status_motor_right = MOTOR_FORWARD;
        status_motor_left = MOTOR_BACKWARD;

        break;
    }

  updateStatusMotor(&(robot->motor_right), status_motor_right);
  updateStatusMotor(&(robot->motor_left), status_motor_left);
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

  /* MCU Configpinsuration--------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */


  MotorPin pin_1_motor_right, pin_2_motor_right;
  Motor motor_right;

  // Motor right is set to the output of the left driver (due to the placement of the driver)
  initMotorPin(&pin_1_motor_right, GPIOB, 9);
  initMotorPin(&pin_2_motor_right, GPIOA, 12);
  initMotor(&motor_right, &pin_1_motor_right, &pin_2_motor_right);

  MotorPin pin_1_motor_left, pin_2_motor_left;
  Motor motor_left;

  // Motor left is set to the output of the right driver (due to the placement of the driver)
  initMotorPin(&pin_1_motor_left, GPIOB, 8);
  initMotorPin(&pin_2_motor_left, GPIOA, 11);
  initMotor(&motor_left, &pin_1_motor_left, &pin_2_motor_left);

  Robot robot;

  initRobot(&robot, &motor_right, &motor_left);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    updateStatusRobot(&robot, ROBOT_FORWARD);
    espera(2);
    updateStatusRobot(&robot, ROBOT_STOPPED);
    espera(1);
    updateStatusRobot(&robot, ROBOT_BACKWARD);
    espera(2);
    updateStatusRobot(&robot, ROBOT_STOPPED);
    espera(1);
    /* USER CODE END WHILE */

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
  HAL_GPIO_WritePin(IDD_CNT_EN_GPIO_Port, IDD_CNT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IDD_CNT_EN_Pin */
  GPIO_InitStruct.Pin = IDD_CNT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IDD_CNT_EN_GPIO_Port, &GPIO_InitStruct);

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
