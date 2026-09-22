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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t position_reference = 0;
uint16_t tx_buff = 0;
uint16_t rx_buff = 0;
char message_received[64];
static float Ep = 0;
static float Ei = 0;
static float Ed = 0;
float error_acc = 0;
float error_prev = 0;
float Ed_prev = 0;
float u_prev = 0;
float Kp = 5;
float Ki = 0.1;
double Kd = 0.35;
float alpha = 0.0075;
float beta = 0.05;
float Ts = 0.0001;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ADC_Init(void);
uint16_t control_action_calc(uint16_t position_reference, uint16_t current_th, float *error_acc, float *error_prev, float *Ed_prev, float *u_prev);
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
  /* USER CODE BEGIN 2 */
  ADC_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_UART_Receive(&huart2, (uint8_t*)&rx_buff, sizeof(rx_buff), HAL_MAX_DELAY) == HAL_OK)
	  {
		  tx_buff = control_action_calc(position_reference, rx_buff, &error_acc, &error_prev, &Ed_prev, &u_prev);
		  HAL_UART_Transmit(&huart2, (uint8_t*)&tx_buff, sizeof(tx_buff), 10);
	  }


//	  int len = sprintf(tx_buff, "%d\r\n", position_reference);
//	  HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, len, 1000);
// 	  HAL_Delay(100);
//	  HAL_UART_Receive(&huart2, (uint8_t*)rx_buff, 64, 1000);
//	  memcpy(message_received, rx_buff, sizeof(rx_buff));
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIOA->MODER |= (3 << (1 * 2)); // pin 1 from GPIO A defined as analog input
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ADC_Init(void)
{
	RCC->APB2ENR |= (1 << 8);	// enabling digital clock for the ADC
	ADC1->CR1 &= ~(3 << 24);	// defining 12-bit resolution
	ADC1->CR1 |= (1 << 5);		// enabling EOC interruption
	ADC1->CR2 |= (1 << 1);		// defining continuous mode
	ADC1->CR2 &= ~(1 << 11);	// right aligned data
	ADC1->SQR1 = 0;				// defining a regular group of 1 conversions
	ADC1->SQR3 = 1;				// channel 1 defined as first in the sequence
	ADC1->SMPR2 |= (3<< 0);		// defining 56 cycles betwwen readings
	ADC1->CR2 |= (1 << 0);		// enabling ADC
	ADC1->CR2 |= (1 << 30);		// starting conversions
	NVIC_EnableIRQ(ADC_IRQn);
}

uint16_t control_action_calc(uint16_t position_reference, uint16_t current_th, float *error_acc, float *error_prev, float *Ed_prev, float *u_prev)
{
	float error = (float)position_reference/217.2465 - (float)current_th/65535*6*3.1416;
	*error_acc += error*Ts;

	Ep = Kp * error;
	Ei = Ki * (*error_acc);
	float Ed_no_filt = Kd * (error - (*error_prev))/Ts;
	Ed = (1.0f - alpha) * (*Ed_prev) + alpha * Ed_no_filt;

	*error_prev = error;
	*Ed_prev = Ed;
	float u_float_filtered = Ep + Ei + Ed;
	float u_float = (1.0f - beta) * (*u_prev) + beta * u_float_filtered;

	if(u_float > 12.0f)
	{
		u_float = 12.0f;
	}
	else if (u_float < -12.0f)
	{
		u_float = -12.0f;
	}

	uint16_t u = (uint16_t)((u_float + 12.0f) * 2730.0f);
	//HAL_UART_Transmit(&huart2, (uint8_t*)&u, sizeof(u), 200);
	return u;
}
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
