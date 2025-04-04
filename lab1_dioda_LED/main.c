/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int program = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
void set_input(uint16_t Pin){
	GPIO_InitTypeDef GPIO_Change = {0};
	GPIO_Change.Pin = Pin;
	GPIO_Change.Mode = GPIO_MODE_INPUT;
	GPIO_Change.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_Change);
}

void set_output(uint16_t LED_Pin){
	GPIO_InitTypeDef GPIO_Change = {0};
	GPIO_Change.Pin = LED_Pin;
	GPIO_Change.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Change.Pull = GPIO_NOPULL;
	GPIO_Change.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_Change);
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
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim9 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  set_output(PB9_Pin | PB6_Pin);
	  set_input(PB8_Pin | PB7_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB9_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, PB6_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB9_Pin | PB7_Pin);
	  set_input(PB6_Pin | PB8_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB9_Pin | PB8_Pin);
	  set_input(PB6_Pin | PB7_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB8_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB9_Pin | PB8_Pin | PB7_Pin | PB6_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB8_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB6_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);


	  set_output(PB8_Pin | PB9_Pin);
	  set_input(PB6_Pin | PB7_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB8_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, PB9_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB8_Pin | PB6_Pin);
	  set_input(PB7_Pin | PB9_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB6_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB8_Pin | PB7_Pin);
	  set_input(PB9_Pin | PB6_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB9_Pin | PB8_Pin | PB7_Pin | PB6_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB9_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB6_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);


	  set_output(PB7_Pin | PB8_Pin);
	  set_input(PB6_Pin | PB9_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, PB8_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB7_Pin | PB9_Pin);
	  set_input(PB8_Pin | PB6_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB9_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB7_Pin | PB6_Pin);
	  set_input(PB9_Pin | PB8_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB6_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB9_Pin | PB8_Pin | PB7_Pin | PB6_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB8_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB9_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB6_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);


	  set_output(PB6_Pin | PB7_Pin);
	  set_input(PB8_Pin | PB9_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB6_Pin | PB8_Pin);
	  set_input(PB9_Pin | PB7_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB8_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB6_Pin | PB9_Pin);
	  set_input(PB7_Pin | PB8_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB9_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);

	  set_output(PB9_Pin | PB8_Pin | PB7_Pin | PB6_Pin);
	  HAL_GPIO_WritePin(GPIOB, PB8_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB7_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, PB9_Pin, GPIO_PIN_RESET);
	  HAL_Delay(300);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 10;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 50000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PB6_GPIO_Port, PB6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6_Pin */
  GPIO_InitStruct.Pin = PB6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PB6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7_Pin PB8_Pin PB9_Pin */
  GPIO_InitStruct.Pin = PB7_Pin|PB8_Pin|PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
