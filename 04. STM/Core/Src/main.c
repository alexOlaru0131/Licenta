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
#include "cmsis_os.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for transmitTask */
osThreadId_t transmitTaskHandle;
const osThreadAttr_t transmitTask_attributes = {
  .name = "transmitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readDistanceTas */
osThreadId_t readDistanceTasHandle;
const osThreadAttr_t readDistanceTas_attributes = {
  .name = "readDistanceTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartFree */
osSemaphoreId_t uartFreeHandle;
const osSemaphoreAttr_t uartFree_attributes = {
  .name = "uartFree"
};
/* Definitions for distanceFlag */
osEventFlagsId_t distanceFlagHandle;
const osEventFlagsAttr_t distanceFlag_attributes = {
  .name = "distanceFlag"
};
/* Definitions for passedSecond */
osEventFlagsId_t passedSecondHandle;
const osEventFlagsAttr_t passedSecond_attributes = {
  .name = "passedSecond"
};
/* USER CODE BEGIN PV */

#define DISTANCE_TOO_CLOSE	(1U << 0)
#define PASSED_SECOND	(1U << 0)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
void StartDefaultTask(void *argument);
void TransmitTask(void *argument);
void ReadDistanceTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t message, timer_counter = 0;

int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
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

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  __HAL_RCC_TIM2_CLK_ENABLE();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_TSENA_Msk;
  ITM->TER = 1;

  InitializeMotorAtts();
  InitializeServo();

	__HAL_RCC_TIM2_CLK_ENABLE();
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uartFree */
  uartFreeHandle = osSemaphoreNew(1, 1, &uartFree_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of transmitTask */
  transmitTaskHandle = osThreadNew(TransmitTask, NULL, &transmitTask_attributes);

  /* creation of readDistanceTas */
  readDistanceTasHandle = osThreadNew(ReadDistanceTask, NULL, &readDistanceTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of distanceFlag */
  distanceFlagHandle = osEventFlagsNew(&distanceFlag_attributes);

  /* creation of passedSecond */
  passedSecondHandle = osEventFlagsNew(&passedSecond_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 79;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 20000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M3_IN1_Pin|M3_IN2_Pin|M4_IN3_Pin|M4_IN4_Pin
                          |M2_IN4_Pin|M2_IN3_Pin|M1_IN2_Pin|M1_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|HC_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_IN1_Pin M3_IN2_Pin M4_IN3_Pin M4_IN4_Pin
                           M2_IN4_Pin M2_IN3_Pin M1_IN2_Pin M1_IN1_Pin */
  GPIO_InitStruct.Pin = M3_IN1_Pin|M3_IN2_Pin|M4_IN3_Pin|M4_IN4_Pin
                          |M2_IN4_Pin|M2_IN3_Pin|M1_IN2_Pin|M1_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin HC_Trig_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|HC_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HC_Echo_Pin */
  GPIO_InitStruct.Pin = HC_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HC_Echo_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

typedef struct SERVO {
	uint16_t SERVO_PIN;
	GPIO_TypeDef* SERVO_PIN_PORT;
} SERVO;

SERVO servo;

volatile uint16_t rotationsM1, rotationsM2, rotationsM3, rotationsM4;

typedef struct MOTOR {
	uint16_t MOTOR_PIN1;
	uint16_t MOTOR_PIN2;
	GPIO_TypeDef* MOTOR_PIN_PORT;

	uint16_t MOTOR_PIN_ENC;
	GPIO_TypeDef* MOTOR_PIN_ENC_PORT;

	uint16_t MOTOR_PIN_PWM;
	GPIO_TypeDef* MOTOR_PIN_PWM_PORT;

	volatile uint16_t MOTOR_ROTATIONS;
} MOTOR;

MOTOR M1, M2, M3, M4;

void InitializeServo()
{
	servo.SERVO_PIN = SERVO_PWM_Pin;
	servo.SERVO_PIN_PORT = SERVO_PWM_GPIO_Port;
}

void InitializeMotorAtts()
{
	M1.MOTOR_PIN1 = M1_IN1_Pin;
	M1.MOTOR_PIN2 = M1_IN2_Pin;
	M1.MOTOR_PIN_PORT = M1_IN1_GPIO_Port;
	M1.MOTOR_PIN_ENC = M1_ENC_Pin;
	M1.MOTOR_PIN_ENC_PORT = M1_ENC_GPIO_Port;
	M1.MOTOR_PIN_PWM = M1_PWM_GEN_Pin;
	M1.MOTOR_PIN_PWM_PORT = M1_PWM_GEN_GPIO_Port;
	M1.MOTOR_ROTATIONS = 0;

	M2.MOTOR_PIN1 = M2_IN3_Pin;
	M2.MOTOR_PIN2 = M2_IN4_Pin;
	M2.MOTOR_PIN_PORT = M2_IN3_GPIO_Port;
	M2.MOTOR_PIN_ENC = M2_ENC_Pin;
	M2.MOTOR_PIN_ENC_PORT = M2_ENC_GPIO_Port;
	M2.MOTOR_PIN_PWM = M2_PWM_GEN_Pin;
	M2.MOTOR_PIN_PWM_PORT = M2_PWM_GEN_GPIO_Port;
	M2.MOTOR_ROTATIONS = 0;

	M3.MOTOR_PIN1 = M3_IN1_Pin;
	M3.MOTOR_PIN2 = M3_IN2_Pin;
	M3.MOTOR_PIN_PORT = M3_IN1_GPIO_Port;
	M3.MOTOR_PIN_ENC = M3_ENC_Pin;
	M3.MOTOR_PIN_ENC_PORT = M3_ENC_GPIO_Port;
	M3.MOTOR_PIN_PWM = M3_PWM_GEN_Pin;
	M3.MOTOR_PIN_PWM_PORT = M3_PWM_GEN_GPIO_Port;
	M3.MOTOR_ROTATIONS = 0;

	M4.MOTOR_PIN1 = M4_IN3_Pin;
	M4.MOTOR_PIN2 = M4_IN4_Pin;
	M4.MOTOR_PIN_PORT = M4_IN3_GPIO_Port;
	M4.MOTOR_PIN_ENC = M4_ENC_Pin;
	M4.MOTOR_PIN_ENC_PORT = M4_ENC_GPIO_Port;
	M4.MOTOR_PIN_PWM = M4_PWM_GEN_Pin;
	M4.MOTOR_PIN_PWM_PORT = M4_PWM_GEN_GPIO_Port;
	M4.MOTOR_ROTATIONS = 0;
}

uint32_t last_tick_ch1 = 0;
uint32_t last_tick_ch2 = 0;
uint32_t last_tick_ch3 = 0;
uint32_t last_tick_ch4 = 0;

#define DEBOUNCE_TIME 1000

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim2);
		switch(htim->Channel)
		{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			if(current_time - last_tick_ch1 >= DEBOUNCE_TIME){
				M1.MOTOR_ROTATIONS++;
				last_tick_ch1 = current_time;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			if(current_time - last_tick_ch2 >= DEBOUNCE_TIME){
				M2.MOTOR_ROTATIONS++;
				last_tick_ch2 = current_time;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			if(current_time - last_tick_ch3 >= DEBOUNCE_TIME){
				M3.MOTOR_ROTATIONS++;
				last_tick_ch3 = current_time;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			if(current_time - last_tick_ch4 >= DEBOUNCE_TIME){
				M4.MOTOR_ROTATIONS++;
				last_tick_ch4 = current_time;
			}
			break;
		default: break;
		}
	}
}

uint8_t received_select[2] = {0, 0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
	osSemaphoreAcquire(uartFreeHandle, 1);

	if (message == 0xAA)
	{
		received_select[0] = 1;
	}
	else if(received_select[0])
	{
    	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

    	    M1.MOTOR_ROTATIONS = 0;
    	    M2.MOTOR_ROTATIONS = 0;
    	    M3.MOTOR_ROTATIONS = 0;
    	    M4.MOTOR_ROTATIONS = 0;

    	    uint8_t PWM_DC_VAL = 0 |
    	    		(message & (1 << 1)) |
    				(message & (1 << 2)) |
    				(message & (1 << 3));
    	    PWM_DC_VAL = PWM_DC_VAL >> 1;
    	    uint16_t pwm_value;
    	    switch(PWM_DC_VAL) {
    			case 0: pwm_value = 0; break;
    	    	case 1: pwm_value = (uint16_t) 65535 * 0.4; break;
    	    	case 2: pwm_value = (uint16_t) 65535 * 0.6; break;
    	    	case 3: pwm_value = (uint16_t) 65535 * 0.8; break;
    	    	case 4: pwm_value = 65535; break;
    	    	default: break;
    	    }

    	    uint8_t MOVE_FORWARD = !(message & (1 << 0));

    	    if (message & (1 << 4)) {
    	    	if (MOVE_FORWARD) {
    	    		HAL_GPIO_WritePin(M1.MOTOR_PIN_PORT, M1.MOTOR_PIN1, GPIO_PIN_RESET);
    				HAL_GPIO_WritePin(M1.MOTOR_PIN_PORT, M1.MOTOR_PIN2, GPIO_PIN_SET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);
    	    	}
    	    	else {
    	    		HAL_GPIO_WritePin(M1.MOTOR_PIN_PORT, M1.MOTOR_PIN1, GPIO_PIN_SET);
    				HAL_GPIO_WritePin(M1.MOTOR_PIN_PORT, M1.MOTOR_PIN2, GPIO_PIN_RESET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);
    	    	}
    	    }
    	    else {
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    	    }

    	    if (message & (1 << 5)) {
    			if (MOVE_FORWARD) {
    				HAL_GPIO_WritePin(M2.MOTOR_PIN_PORT, M2.MOTOR_PIN1, GPIO_PIN_RESET);
    				HAL_GPIO_WritePin(M2.MOTOR_PIN_PORT, M2.MOTOR_PIN2, GPIO_PIN_SET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);
    			}
    			else {
    				HAL_GPIO_WritePin(M2.MOTOR_PIN_PORT, M2.MOTOR_PIN1, GPIO_PIN_SET);
    				HAL_GPIO_WritePin(M2.MOTOR_PIN_PORT, M2.MOTOR_PIN2, GPIO_PIN_RESET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);
    			}
    		}
    		else {
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    		}

    		if (message & (1 << 6)) {
    			if (MOVE_FORWARD) {
    				HAL_GPIO_WritePin(M3.MOTOR_PIN_PORT, M3.MOTOR_PIN1, GPIO_PIN_RESET);
    				HAL_GPIO_WritePin(M3.MOTOR_PIN_PORT, M3.MOTOR_PIN2, GPIO_PIN_SET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_value);
    			}
    			else {
    				HAL_GPIO_WritePin(M3.MOTOR_PIN_PORT, M3.MOTOR_PIN1, GPIO_PIN_SET);
    				HAL_GPIO_WritePin(M3.MOTOR_PIN_PORT, M3.MOTOR_PIN2, GPIO_PIN_RESET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_value);
    			}
    		}
    		else {
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    		}

    		if (message & (1 << 7)) {
    			if (MOVE_FORWARD) {
    				HAL_GPIO_WritePin(M4.MOTOR_PIN_PORT, M4.MOTOR_PIN1, GPIO_PIN_RESET);
    				HAL_GPIO_WritePin(M4.MOTOR_PIN_PORT, M4.MOTOR_PIN2, GPIO_PIN_SET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_value);
    			}
    			else {
    				HAL_GPIO_WritePin(M4.MOTOR_PIN_PORT, M4.MOTOR_PIN1, GPIO_PIN_SET);
    				HAL_GPIO_WritePin(M4.MOTOR_PIN_PORT, M4.MOTOR_PIN2, GPIO_PIN_RESET);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value);
    				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_value);
    			}
    		}
    		else {
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    		}

    		received_select[0] = 0;
    	}

	if (message == 0xAB)
	{
		received_select[1] = 1;
	}
	else if(received_select[1])
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

			uint8_t PWM_DC_VAL = 0 |
					(message & (1 << 0)) |
					(message & (1 << 1)) |
					(message & (1 << 2)) |
					(message & (1 << 3));

			uint16_t pwm_value;
			if(PWM_DC_VAL == 0) pwm_value = 1000;
			else pwm_value = 1000 + 100 * PWM_DC_VAL;
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pwm_value);

    		received_select[1] = 0;
    	}

    	HAL_UART_Receive_IT(&huart4, &message, 1);
    	osSemaphoreRelease(uartFreeHandle);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

static TickType_t last_time = 0;

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	HAL_UART_Receive_IT(&huart4, &message, 1);

  /* Infinite loop */
	  for(;;)
	  {

		  if(xTaskGetTickCount() - last_time >= 1000)
		  {
			  last_time += 1000;
			  osEventFlagsSet(passedSecondHandle, PASSED_SECOND);

			  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		  }

		  osDelay(1);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TransmitTask */
/**
* @brief Function implementing the transmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransmitTask */
void TransmitTask(void *argument)
{
  /* USER CODE BEGIN TransmitTask */
  /* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(uartFreeHandle, 10);

		rotationsM1 = M1.MOTOR_ROTATIONS;
		rotationsM2 = M2.MOTOR_ROTATIONS;
		rotationsM3 = M3.MOTOR_ROTATIONS;
		rotationsM4 = M4.MOTOR_ROTATIONS;

		uint32_t flagDistance = osEventFlagsGet(distanceFlagHandle);
		uint32_t flagSecond = osEventFlagsGet(passedSecondHandle);

		if(flagDistance & DISTANCE_TOO_CLOSE)
		{
			uint8_t tx[1];
			tx[0] = 0xFA;
			HAL_UART_Transmit(&huart4, tx, 1, 10);
			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		}
		else if(flagSecond & PASSED_SECOND){
			uint8_t tx[12];
			tx[0] = 0xAA;
			tx[1] = rotationsM1 & 0xFF;
			tx[2] = (rotationsM1 >> 8) & 0xFF;
			tx[3] = 0xAB;
			tx[4] = rotationsM2 & 0xFF;
			tx[5] = (rotationsM2 >> 8) & 0xFF;
			tx[6] = 0xAC;
			tx[7] = rotationsM3 & 0xFF;
			tx[8] = (rotationsM3 >> 8) & 0xFF;
			tx[9] = 0xAD;
			tx[10] = rotationsM4 & 0xFF;
			tx[11] = (rotationsM4 >> 8) & 0xFF;
			HAL_UART_Transmit(&huart4, tx, 12, 10);

			osEventFlagsClear(passedSecondHandle, PASSED_SECOND);
			M1.MOTOR_ROTATIONS = 0;
			M2.MOTOR_ROTATIONS = 0;
			M3.MOTOR_ROTATIONS = 0;
			M4.MOTOR_ROTATIONS = 0;

			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		}

		osDelay(100);
		osSemaphoreRelease(uartFreeHandle);
	}
  /* USER CODE END TransmitTask */
}

/* USER CODE BEGIN Header_ReadDistanceTask */
/**
* @brief Function implementing the readDistanceTas thread.
* @param argument: Not used
* @retval None
*/

#define CONVERT_TO_DISTANCE(duration) ((float)(duration*0.0343)/2)

typedef struct HC_SR_04 {
	uint16_t Echo_Pin;
	GPIO_TypeDef* Echo_Pin_PORT;

	uint16_t Trig_Pin;
	GPIO_TypeDef* Trig_Pin_PORT;

	float LAST_DISTANCE_MEASURED;
} HC_SR_04;

HC_SR_04 sensor = {HC_Echo_Pin, HC_Echo_GPIO_Port, HC_Trig_Pin, HC_Trig_GPIO_Port, 0};

/* USER CODE END Header_ReadDistanceTask */
void ReadDistanceTask(void *argument)
{
  /* USER CODE BEGIN ReadDistanceTask */
	uint32_t cnt;
	__HAL_RCC_TIM3_CLK_ENABLE();
	HAL_TIM_Base_Start(&htim3);
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_WritePin(sensor.Trig_Pin_PORT, sensor.Trig_Pin, GPIO_PIN_RESET);
		osDelay(2);
		HAL_GPIO_WritePin(sensor.Trig_Pin_PORT, sensor.Trig_Pin, GPIO_PIN_SET);
		osDelay(10);
		HAL_GPIO_WritePin(sensor.Trig_Pin_PORT, sensor.Trig_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COUNTER(&htim3, 0);

		uint32_t t0 = __HAL_TIM_GET_COUNTER(&htim3);
		while(HAL_GPIO_ReadPin(sensor.Echo_Pin_PORT, sensor.Echo_Pin) == GPIO_PIN_RESET)
		{
			if((__HAL_TIM_GET_COUNTER(&htim3) - t0) > 30000)
				goto end;
		}

		__HAL_TIM_SET_COUNTER(&htim3, 0);
		while(HAL_GPIO_ReadPin(sensor.Echo_Pin_PORT, sensor.Echo_Pin) == GPIO_PIN_SET);

		cnt = __HAL_TIM_GET_COUNTER(&htim3);
		sensor.LAST_DISTANCE_MEASURED = CONVERT_TO_DISTANCE(cnt);

		if(sensor.LAST_DISTANCE_MEASURED < 20)
		{
			osEventFlagsSet(distanceFlagHandle, DISTANCE_TOO_CLOSE);
		}
		else
		{
			osEventFlagsClear(distanceFlagHandle, DISTANCE_TOO_CLOSE);
		}

		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

		end:
			osDelay(60);
	}
  /* USER CODE END ReadDistanceTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
