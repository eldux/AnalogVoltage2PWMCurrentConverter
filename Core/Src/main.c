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
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_COEF_UPPER_LIMIT (1020) // That is from
#define PWM_COEF_LOWER_LIMIT (980)
#define PWM_MAXIMUM_LIMIT (900.0f)
#define PWM_MAXIMUM_LIMIT_UNCONNECTED (300.0f)

#define LIMIT_CURRENT_MA (1600)
#define NOMINAL_VOLTAGE_MV (12000)
#define LOAD_RESISTANCE_OHM (4.8f)
#define MAX_POSSIBLE_CURRENT_MA (uint16_t)(NOMINAL_VOLTAGE_MV/LOAD_RESISTANCE_OHM)///(5000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t console_buffer[100] = {0};
uint8_t print_buf[100] = {0};
uint8_t console_msg_lenght = 0;

float pwm_f = 0;
uint8_t input_use = 1;
uint8_t output_on = 1;
uint8_t debug_on = 0;
uint16_t adc1_buffer[8000] = {0};
uint16_t adc2_buffer[20] = {0};
uint32_t output_current_acc;
uint32_t input_voltage_acc;
int16_t output_current_mA = 0;
int16_t input_voltage_mV = 0;
int16_t required_current_mA = 0;
uint16_t pwm = 0;
int16_t input_voltage_raw = 0;
int16_t output_current_raw = 0;
float offset = 40;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Console_Parser(uint8_t * console_buffer, uint8_t console_msg_lenght);
void print(char* fmt,...);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  //Calibrate ADCs
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  //Start ADC1 in DMA mode
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1_buffer[0], 8000);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc2_buffer[0], 20);
  HAL_UART_Receive_IT(&huart2, &console_buffer[console_msg_lenght], 1);
  print("dSpace converter 1.0\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	if (debug_on)
	{
		print("Input: %d mV, Output: %s, Req.cur: %d mA, Meas.cur.: %d mA, Set by: %s\r\n", input_voltage_mV, (output_on)? "on" : "off", required_current_mA, output_current_mA, (input_use)? "input req." : "user req.");
		HAL_Delay(200);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV17;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
	{
		uint16_t i = 0;
		output_current_acc = 0;
		for (i = 0; i < 8000; i++)
		{
			output_current_acc += adc1_buffer[i];
		}
		output_current_raw = (int16_t)(((float)output_current_acc)/8000.0f - offset);
		if ((required_current_mA == 0)&&(pwm == 0))
		{
			offset += output_current_raw/10.0f;
			if (offset < 0.0f)
			{
				offset = 0.0f;
			}
			else if (offset > 50.0f) offset = 50.0f;
		}
		if (output_current_raw < 0) output_current_raw = 0;
		output_current_mA = output_current_raw * 0.81f / 1.40f;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1_buffer[0], 8000);

		int16_t current_error = output_current_mA - required_current_mA;
		float error_correction = ((float)current_error) / 10.0f;
		if (error_correction > 10.0f)
		{
			error_correction = 10.0f;
		}
		else if (error_correction < -10.0f)
		{
			error_correction = -10.0f;
		}
		if (error_correction > 1.0f)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
		else if (error_correction < -1.0f)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
		pwm_f -= error_correction;
		if (pwm_f < 0.0f) pwm_f = 0.0f;
		if (pwm_f > PWM_MAXIMUM_LIMIT) pwm_f = PWM_MAXIMUM_LIMIT;
		if ((output_current_raw < 50)&&(pwm_f > PWM_MAXIMUM_LIMIT_UNCONNECTED)) pwm_f = PWM_MAXIMUM_LIMIT_UNCONNECTED;
		if (required_current_mA == 0)
		{
			pwm_f = 0.0f;
		}
		pwm = (uint16_t)pwm_f;
		if (output_on) TIM3->CCR2 = pwm;
		else TIM3->CCR2 = 0;
	}
	else if (hadc->Instance == ADC2)
	{
		uint16_t i = 0;
		input_voltage_acc = 0;
		for (i = 0; i < 20; i++)
		{
			input_voltage_acc += adc2_buffer[i];
		}
		input_voltage_raw = input_voltage_acc/20 - 8;
		if (input_voltage_raw < 0) input_voltage_raw = 0;
		input_voltage_mV = (uint16_t)(input_voltage_raw * 0.8056f * 3.44f);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc2_buffer[0], 20);

		uint16_t required_current_mA_internal = (uint16_t)((LIMIT_CURRENT_MA * input_voltage_mV) / 10000);
		if (required_current_mA_internal > LIMIT_CURRENT_MA) required_current_mA_internal = LIMIT_CURRENT_MA;
		// Just for testing
		if (input_use) required_current_mA = required_current_mA_internal;
	}
	return;
}

void print(char* fmt,...)
{
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf((char*)print_buf, 100, fmt, args);
	va_end(args);
	HAL_UART_Transmit(&huart2, (uint8_t*)print_buf, len, 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if ((console_buffer[console_msg_lenght] == 13) || (console_buffer[console_msg_lenght] == 10))
		{
			print("\r\n");
			console_buffer[console_msg_lenght] = 0;
			Console_Parser(&console_buffer[0], console_msg_lenght);
			console_msg_lenght = 0;
			HAL_UART_Receive_IT(&huart2, &console_buffer[console_msg_lenght], 1);
		}
		else if (console_buffer[console_msg_lenght] == 8)
		{
			print("\b \b");
			console_buffer[console_msg_lenght] = 0;
			if (console_msg_lenght > 0) console_msg_lenght--;
			HAL_UART_Receive_IT(&huart2, &console_buffer[console_msg_lenght], 1);
		}
		else
		{
			HAL_UART_Transmit(&huart2, &console_buffer[console_msg_lenght], 1, 10);
			if (console_msg_lenght < 100) console_msg_lenght++;
			else console_msg_lenght = 0;
			HAL_UART_Receive_IT(&huart2, &console_buffer[console_msg_lenght], 1);
		}
	}
}

void Console_Parser(uint8_t * console_buffer, uint8_t console_msg_lenght)
{
	if (strncmp((char*)console_buffer, "on", 2) == 0)
	{
		print("Output on\r\n");
		output_on = 1;
	}
	else if (strncmp((char*)console_buffer, "off", 3) == 0)
	{
		print("Output off\r\n");
		output_on = 0;
	}
	else if (strncmp((char*)console_buffer, "reset", 5) == 0)
	{
		print("Reseting\r\n");
		HAL_NVIC_SystemReset();
	}
	else if (strncmp((char*)console_buffer, "set", 3) == 0)
	{
		int current = 0;
		if (sscanf ((char*)console_buffer, "set %d", &current) == 1)
		{
			if ((current >= 0)&&(current < LIMIT_CURRENT_MA))
			{
				input_use = 0;
				required_current_mA = current;
				print("Set requested current to %d mA. Auto off. \r\n", required_current_mA);
			}
			else
			{
				print("Out of range, please input between 0 and %d\r\n", LIMIT_CURRENT_MA);
			}
		}
		else
		{
			print("Syntax error. Expected format \"set <current_ma>\"\r\n");
		}
	}
	else if (strncmp((char*)console_buffer, "get", 3) == 0)
	{
		print("Req.cur: %d mA, Meas.cur.: %d mA\r\n", required_current_mA, output_current_mA);
	}
	else if (strncmp((char*)console_buffer, "info", 4) == 0)
	{
		print("Input: %d mV, Output: %s, Req.cur: %d mA, Meas.cur.: %d mA, Set by: %s\r\n", input_voltage_mV, (output_on)? "on" : "off", required_current_mA, output_current_mA, (input_use)? "input req." : "user req.");
	}
	else if (strncmp((char*)console_buffer, "auto", 4) == 0)
	{
		print("Auto mode enabled\r\n");
		input_use = 1;
		output_on = 1;
	}
	else if (strncmp((char*)console_buffer, "debug on", 8) == 0)
	{
		print("Debug print on\r\n");
		debug_on = 1;
	}
	else if (strncmp((char*)console_buffer, "debug off", 9) == 0)
	{
		print("Debug print off\r\n");
		debug_on = 0;
	}
	else if (strncmp((char*)console_buffer, "help", 4) == 0)
	{
		print("Commands list:\r\n");
		print(" help - prints command list with explanations\r\n");
		print(" on   - enables output (default is on)\r\n");
		print(" off  - disables output \r\n");
		print(" set <current_ma> - sets current in limits from 0 to %d\r\n", LIMIT_CURRENT_MA);
		print(" get - prints requested and measured currents\r\n");
		print(" info - prints all current settings and measurements\r\n");
		print(" auto - returns to automatic output control by input\r\n");
		print(" reset - resets the converter box\r\n");
		print(" debug <on/off> - turn on/off periodic debug info print\r\n");
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
