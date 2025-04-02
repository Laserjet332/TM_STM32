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
int current_segment = 0;
int current_time = 0;
const int one_second = 100;
int number_com1 = 0;
int number_com2 = 0;
int number_com3 = 0;
int number_com4 = 0;
const uint16_t NUMBERS[] = {((uint16_t)0x007E),((uint16_t)0x0018),((uint16_t)0x0057),((uint16_t)0x005B),
		((uint16_t)0x0039),((uint16_t)0x006B),((uint16_t)0x006F),((uint16_t)0x0058),
		((uint16_t)0x007F),((uint16_t)0x007B)};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim9){
		if (current_segment==3){
			current_segment = 0;
		}
		else{
			current_segment+=1;
		}
		if (current_time == one_second){
			current_time=0;
			number_com1+=1;
		}
		else{
			current_time+=1;
		}
	}
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
  while (1){
	  switch(current_segment){
	  	  case 0:
	  		  HAL_GPIO_WritePin(GPIOC, ((uint16_t)0x00FF), GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM1_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM2_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM3_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM4_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, NUMBERS[number_com4], GPIO_PIN_SET);
	  		  break;
	  	  case 1:
	  		  HAL_GPIO_WritePin(GPIOC, ((uint16_t)0x00FF), GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM1_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM2_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM3_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM4_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, NUMBERS[number_com3], GPIO_PIN_SET);
	  		  break;
	  	  case 2:
	  		  HAL_GPIO_WritePin(GPIOC, ((uint16_t)0x00FF), GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM1_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM2_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM3_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM4_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, NUMBERS[number_com2], GPIO_PIN_SET);
	  		  break;
	  	  case 3:
	  		  HAL_GPIO_WritePin(GPIOC, ((uint16_t)0x00FF), GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM1_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM2_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM3_Pin,GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOC, S7COM4_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOC, NUMBERS[number_com1], GPIO_PIN_SET);
	  		  break;
	  	  default:
	  		  break;
	  }

	  if(number_com1==10){
		  number_com1 = 0;
		  number_com2+=1;
	  }
	  if(number_com2==10){
		  number_com2 = 0;
		  number_com3+=1;
	  }
	  if(number_com3==10){
		  number_com3 = 0;
		  number_com4+=1;
	  }
	  if(number_com4==10){
		  number_com4 = 0;
		  number_com1+=1;
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOC, S7G_Pin|S7D_Pin|S7E_Pin|S7C_Pin
                          |S7B_Pin|S7F_Pin|S7A_Pin|S7DP_Pin
                          |S7COM4_Pin|S7COM3_Pin|S7COM2_Pin|S7COM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S7G_Pin S7D_Pin S7E_Pin S7C_Pin
                           S7B_Pin S7F_Pin S7A_Pin S7DP_Pin
                           S7COM4_Pin S7COM3_Pin S7COM2_Pin S7COM1_Pin */
  GPIO_InitStruct.Pin = S7G_Pin|S7D_Pin|S7E_Pin|S7C_Pin
                          |S7B_Pin|S7F_Pin|S7A_Pin|S7DP_Pin
                          |S7COM4_Pin|S7COM3_Pin|S7COM2_Pin|S7COM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
