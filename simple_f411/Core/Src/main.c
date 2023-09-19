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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CK_CNT 1000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId ProgramingModeHandle;
osThreadId InterpreterHandle;
osThreadId UART_comunicationHandle;
osThreadId ExecutionFromMemoryHandle;
/* USER CODE BEGIN PV */
QueueHandle_t uart_queue_rx;
QueueHandle_t programing_queue;
//QueueHandle_t instruction_S_value;

struct exm_type{
	char execution_memory[119]; //Program storage capability of GS-C200S
	char* memory_pointer;
	uint8_t sizes_of_instruction[60];
	uint8_t* sizes_pointer;
	char current_instruction[4];
	uint16_t start_speed_value;
	uint16_t top_speed_value;
	uint16_t ramp_value;
	uint16_t G_sign_value;
	uint16_t step_counter_second_part;
	int16_t position;  // 0 is home position
}exm;


EventGroupHandle_t EventGroup;
/* Event Group description
 *  0x80 Programming mode for GS-200S
 *	0x40 Execution mode
 *	0x10 Delay until...
 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
void programing_mode(void const * argument);
void interpreter(void const * argument);
void uart_comunication(void const * argument);
void execution_from_memory(void const * argument);

/* USER CODE BEGIN PFP */
void uart1_rx_callback();
void enter_programing();
void exit_programing();
void write_to_exm(uint8_t* start_of_data, uint8_t size_of_data);
void read_from_exm();
void erase_exm();
void start_motor();
void speed_regulation();
void stop_motor();

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

	for(int i = 0; i < sizeof(exm.execution_memory); i++){
		exm.execution_memory[i] = 0;
	}
	exm.memory_pointer = exm.execution_memory;
	for(int i = 0; i < sizeof(exm.sizes_of_instruction); i++){
		exm.sizes_of_instruction[i] = 1;
	}
	exm.sizes_pointer = exm.sizes_of_instruction;
	for(int i = 0; i < sizeof(exm.current_instruction); i++){
		exm.current_instruction[i] = 0;
	}
	exm.start_speed_value = 0;
	exm.top_speed_value = 0;
	exm.ramp_value = 0;
	exm.G_sign_value = 0;
	exm.step_counter_second_part = 0;

	exm.position = 0; // temporary
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  USART1->CR1 |= USART_CR1_RXNEIE;
  USART1->CR1 |= USART_CR1_TE;
  USART1->CR1 |= USART_CR1_RE;
  USART1->CR1 |= USART_CR1_UE; //USART enable

  GPIOB->ODR &= ~GPIO_ODR_OD10; //Enable stepper driver
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  EventGroup = xEventGroupCreate();
  xEventGroupSetBits(EventGroup, 0x40); // Start the Execution mode
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  uart_queue_rx = xQueueCreate(32, 1);
  programing_queue = xQueueCreate(64, 1);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ProgramingMode */
  osThreadDef(ProgramingMode, programing_mode, osPriorityNormal, 0, 500);
  ProgramingModeHandle = osThreadCreate(osThread(ProgramingMode), NULL);

  /* definition and creation of Interpreter */
  osThreadDef(Interpreter, interpreter, osPriorityNormal, 0, 500);
  InterpreterHandle = osThreadCreate(osThread(Interpreter), NULL);

  /* definition and creation of UART_comunication */
  osThreadDef(UART_comunication, uart_comunication, osPriorityNormal, 0, 160);
  UART_comunicationHandle = osThreadCreate(osThread(UART_comunication), NULL);

  /* definition and creation of ExecutionFromMemory */
  osThreadDef(ExecutionFromMemory, execution_from_memory, osPriorityNormal, 0, 1000);
  ExecutionFromMemoryHandle = osThreadCreate(osThread(ExecutionFromMemory), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_ODD;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, MOV_Pin|RAMP_Pin|RDY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|E_MUX_Pin|ENA_stepper_Pin|S0_MUX_Pin
                          |S1_MUX_Pin|S2_MUX_Pin|S3_MUX_Pin|DIR_stepper_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HOME_Pin MOVE_EN_Pin */
  GPIO_InitStruct.Pin = HOME_Pin|MOVE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOV_Pin */
  GPIO_InitStruct.Pin = MOV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RAMP_Pin RDY_Pin */
  GPIO_InitStruct.Pin = RAMP_Pin|RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : REC_Pin */
  GPIO_InitStruct.Pin = REC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(REC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : E_MUX_Pin */
  GPIO_InitStruct.Pin = E_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(E_MUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENA_stepper_Pin DIR_stepper_Pin */
  GPIO_InitStruct.Pin = ENA_stepper_Pin|DIR_stepper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_MUX_Pin S1_MUX_Pin S2_MUX_Pin S3_MUX_Pin */
  GPIO_InitStruct.Pin = S0_MUX_Pin|S1_MUX_Pin|S2_MUX_Pin|S3_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COM_IO_MUX_Pin */
  GPIO_InitStruct.Pin = COM_IO_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COM_IO_MUX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void uart1_rx_callback(void)
{
	xQueueSendToBackFromISR(uart_queue_rx, (void*)&(USART1->DR), pdTRUE);
	__asm__ volatile("NOP");

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0) // HOME position
	{
		HAL_UART_Transmit(&huart1, "HOME", 5, 100);
		exm.position = 0;
	}
}

void enter_programing()
{
//	xEventGroupClearBits(EventGroup, 0x40);
	xEventGroupClearBits(EventGroup, 0xFFFFFF);
	exm.memory_pointer = exm.execution_memory;
	exm.sizes_pointer = exm.sizes_of_instruction;
	erase_exm();
	xEventGroupSetBits(EventGroup, 0x80);
}

void exit_programing()
{
//	xEventGroupClearBits(EventGroup, 0x80);
	xEventGroupClearBits(EventGroup, 0xFFFFFF);
	exm.memory_pointer = exm.execution_memory;
	exm.sizes_pointer = exm.sizes_of_instruction;

	exm.start_speed_value = 0;
	exm.top_speed_value = 0;
	exm.ramp_value = 0;
	exm.G_sign_value = 0;

	xEventGroupSetBits(EventGroup, 0x40);
}

void write_to_exm(uint8_t* start_of_data, uint8_t size_of_data)
{
	for(int i = 0; i < size_of_data; i++){
//		exm.execution_memory[exm.memory_pointer] = start_of_data[i];
		*exm.memory_pointer = start_of_data[i];
		exm.memory_pointer++;
		if(exm.memory_pointer >= (exm.execution_memory + 119)){
			exm.memory_pointer = exm.execution_memory + 118;
			return;
		}
	}
//	exm.sizes_of_instruction[exm.sizes_pointer] = size_of_data;
	*exm.sizes_pointer = size_of_data;
	exm.sizes_pointer++;
	if(exm.sizes_pointer >= (exm.sizes_of_instruction + 60)){
		exm.sizes_pointer = exm.sizes_of_instruction + 59;
		return;
	}
}

void read_from_exm()
{
	for(int i = 0; i < 4; i++){
		exm.current_instruction[i] = 0;
	}
	for(int i = 0; i < *exm.sizes_pointer; i++){
		exm.current_instruction[i] = *exm.memory_pointer;
		exm.memory_pointer++;
		if(exm.memory_pointer >= (exm.execution_memory + 119)){
			exm.memory_pointer = exm.execution_memory + 118;
			return ;
		}
	}
	exm.sizes_pointer++;
	if(exm.sizes_pointer >= (exm.sizes_of_instruction + 60)){
		exm.sizes_pointer = exm.sizes_of_instruction + 59;
		return ;
	}
	return ;
}

void erase_exm()
{
	for(int i = 0; i < sizeof(exm.execution_memory); i++){
		exm.execution_memory[i] = 0;
	}
	for(int i = 0; i < sizeof(exm.sizes_of_instruction); i++){
		exm.sizes_of_instruction[i] = 1;
	}
}

void start_motor()
{
	if(exm.start_speed_value == 0) return;
	HAL_TIM_Base_Start_IT(&htim4);
	uint16_t temp = (uint16_t)(CK_CNT / exm.start_speed_value);
	TIM4->ARR = temp;
	TIM4->CCR1 = temp/2;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void speed_regulation()
{

}

void stop_motor()
{

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_programing_mode */
/**
  * @brief  Function implementing the ProgramingMode thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_programing_mode */
void programing_mode(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  xEventGroupWaitBits(EventGroup, 0x80, pdFALSE, pdTRUE, portMAX_DELAY);
	  HAL_UART_Transmit(&huart1, "Programming mode", 17, 100);
	  //vTaskDelay(200);
	  char instruction_for_programing = 0;
	  xQueueReceive(programing_queue, &instruction_for_programing, portMAX_DELAY);
	  switch (instruction_for_programing) {
		case 'S':
			uint8_t data1[4] = {'S', (uint8_t)(exm.start_speed_value), (uint8_t)((exm.start_speed_value>>8)), 0};
			write_to_exm(data1, sizeof(data1));
			break;
		case 'T':
			uint8_t data2[4] = {'T', (uint8_t)(exm.top_speed_value), (uint8_t)((exm.top_speed_value>>8)), 0};
			write_to_exm(data2, sizeof(data2));
			break;
		case 'R':
			uint8_t data3[4] = {'R', (uint8_t)(exm.ramp_value), (uint8_t)((exm.ramp_value>>8)), 0};
			write_to_exm(data3, sizeof(data3));
			break;
		case 'G':
			instruction_for_programing = 0;
			  xQueueReceive(programing_queue, &instruction_for_programing, 5);
			  switch (instruction_for_programing) {
			  	  case '+':
						uint8_t data4[4] = {('G'+'+'), (uint8_t)(exm.G_sign_value), (uint8_t)((exm.G_sign_value>>8)), (uint8_t)((exm.G_sign_value>>16))};
						write_to_exm(data4, sizeof(data4));
			  		  break;
			  	  case '-':
						uint8_t data5[4] = {('G'+'-'), (uint8_t)(exm.G_sign_value), (uint8_t)((exm.G_sign_value>>8)), (uint8_t)((exm.G_sign_value>>16))};
						write_to_exm(data5, sizeof(data5));
			  		  break;
			  }
			break;
		default:
			break;
	}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_interpreter */
/**
* @brief Function implementing the Interpreter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_interpreter */
void interpreter(void const * argument)
{
  /* USER CODE BEGIN interpreter */
	char pData;
  /* Infinite loop */
  for(;;)
  {
	  xQueueReceive(uart_queue_rx, &pData, portMAX_DELAY);
	  HAL_UART_Transmit(&huart1, &pData, 1, 10);
	  if ((pData & (1<<7)) == 0x80){  //Check odd parity
		  pData &= ~(1<<7);
	  }
	  __asm__ volatile("NOP");
	  char temp = 0;
	  switch (pData) {
		case 'P':
			xQueueReceive(uart_queue_rx, &pData, 5);
			HAL_UART_Transmit(&huart1, &pData, 1, 10);
				  if ((pData & (1<<7)) == 0x80){ //Check odd parity
					  pData &= ~(1<<7);
				  }
			__asm__ volatile("NOP");
			switch (pData) {
				case 'o':
					enter_programing();
					break;
				case 'x':
					exit_programing();
					break;
				default:
					break;
			}
			break;
	    case 'E':
	    	exit_programing();
	    	break;
		case 'S':
			if((xEventGroupGetBits(EventGroup) & (1<<7)) != 0x80) break; // if  Programming mode is OFF
			exm.start_speed_value = 0;
			temp = 0;
			for(int i = 0; i < 5; i++){
				xQueueReceive(uart_queue_rx, &temp, 5);
				if((temp == 13) || (temp == 0) || (i == 4)) break;
				if ((temp & (1<<7)) == 0x80){ //Check odd parity
					temp &= ~(1<<7);
			    }
				exm.start_speed_value = (exm.start_speed_value*10) + (temp - 48);
			}
			xQueueSendToBack(programing_queue, (void*)"S", 100);
			//exm.
			__asm__ volatile("NOP");
			break;
		case 'T':
			if((xEventGroupGetBits(EventGroup) & (1<<7)) != 0x80) break; // if  Programming mode is OFF
			exm.top_speed_value = 0;
			temp = 0;
			for(int i = 0; i < 4; i++){
				xQueueReceive(uart_queue_rx, &temp, 5);
				if((temp == 13) || (temp == 0) || (i == 3)) break;
				if(i == 3) break;
				if ((temp & (1<<7)) == 0x80){ //Check odd parity
					temp &= ~(1<<7);
			    }
				exm.top_speed_value = (exm.top_speed_value*10) + (temp - 48);
			}
			xQueueSendToBack(programing_queue, (void*)"T", 100);
			__asm__ volatile("NOP");
			break;
		case 'R':
			if((xEventGroupGetBits(EventGroup) & (1<<7)) != 0x80) break; // if  Programming mode is OFF
			exm.ramp_value = 0;
			temp = 0;
			for(int i = 0; i < 4; i++){
				xQueueReceive(uart_queue_rx, &temp, 5);
				if((temp == 13) || (temp == 0) || (i == 3)) break;
				if(i == 3) break;
				if ((temp & (1<<7)) == 0x80){ //Check odd parity
					temp &= ~(1<<7);
			    }
				exm.ramp_value = (exm.ramp_value*10) + (temp - 48);
			}
			xQueueSendToBack(programing_queue, (void*)"R", 100);
			__asm__ volatile("NOP");
			break;
		case 'G':
			if((xEventGroupGetBits(EventGroup) & (1<<7)) != 0x80) break; // if  Programming mode is OFF
			xQueueReceive(uart_queue_rx, &pData, 5);
			  if ((pData & (1<<7)) == 0x80){ //Check odd parity
				  pData &= ~(1<<7);
			  }
			__asm__ volatile("NOP");
			switch (pData) {
				case '+':
					exm.G_sign_value = 0;
					temp = 0;
					for(int i = 0; i < 8; i++){
						xQueueReceive(uart_queue_rx, &temp, 5);
						if((temp == 13) || (temp == 0) || (i == 7)) break;
						if ((temp & (1<<7)) == 0x80){ //Check odd parity
							temp &= ~(1<<7);
					    }
						exm.G_sign_value = (exm.G_sign_value*10) + (temp - 48);
					}
					xQueueSendToBack(programing_queue, (void*)"G", 100);
					xQueueSendToBack(programing_queue, (void*)"+", 100);
					__asm__ volatile("NOP");
					break;
				case '-':
					exm.G_sign_value = 0;
					temp = 0;
					for(int i = 0; i < 8; i++){
						xQueueReceive(uart_queue_rx, &temp, 5);
						if((temp == 13) || (temp == 0) || (i == 7)) break;
						if ((temp & (1<<7)) == 0x80){ //Check odd parity
							temp &= ~(1<<7);
					    }
						exm.G_sign_value = (exm.G_sign_value*10) + (temp - 48);
					}
					xQueueSendToBack(programing_queue, (void*)"G", 100);
					xQueueSendToBack(programing_queue, (void*)"-", 100);
					__asm__ volatile("NOP");
					break;
				default:
					break;
			}
			break;
	    case 0:
			break;
		default:
			break;
	}

	  pData = 0;
  }
  /* USER CODE END interpreter */
}

/* USER CODE BEGIN Header_uart_comunication */
/**
* @brief Function implementing the UART_comunication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_comunication */
void uart_comunication(void const * argument)
{
  /* USER CODE BEGIN uart_comunication */
//	GPIOB->ODR |= GPIO_ODR_OD9; //counterclockwise
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
	  EventBits_t res = xEventGroupGetBits(EventGroup); // EventBits_t is uint32_t
//	  HAL_UART_Transmit(&huart1, &res, 4, 100);
	  vTaskDelay(1500);

  }
  /* USER CODE END uart_comunication */
}

/* USER CODE BEGIN Header_execution_from_memory */
/**
* @brief Function implementing the ExecutionFromMemory thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_execution_from_memory */
void execution_from_memory(void const * argument)
{
  /* USER CODE BEGIN execution_from_memory */
  /* Infinite loop */
  for(;;)
  {
	  xEventGroupWaitBits(EventGroup, 0x40, pdFALSE, pdTRUE, portMAX_DELAY);
	  HAL_UART_Transmit(&huart1, "Execution mode", 15, 100);
	  read_from_exm();
	  HAL_UART_Transmit(&huart1, exm.current_instruction, sizeof(exm.current_instruction), 100);
	  switch (exm.current_instruction[0]) {
		case 'S':
			exm.start_speed_value = 0;
			exm.start_speed_value = exm.current_instruction[1] + (exm.current_instruction[2]<<8);
			break;
		case 'T':
			exm.top_speed_value = 0;
			exm.top_speed_value = exm.current_instruction[1] + (exm.current_instruction[2]<<8);
			break;
		case 'R':
			exm.ramp_value = 0;
			exm.ramp_value = exm.current_instruction[1] + (exm.current_instruction[2]<<8);
			break;
		case ('G'+'+'):
			exm.G_sign_value = 0;
			exm.G_sign_value = exm.current_instruction[1] + (exm.current_instruction[2]<<8) + (exm.current_instruction[2]<<16);
			if(exm.position == exm.G_sign_value) break;
			else if(exm.position > exm.G_sign_value){
				GPIOB->ODR |= GPIO_ODR_OD9; //counterclockwise
			}else{
				GPIOB->ODR &= ~GPIO_ODR_OD9; //clockwise
			}
//			  HAL_TIM_Base_Start_IT(&htim4);
//			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			if(exm.start_speed_value == 0) break;
			start_motor();
			xEventGroupWaitBits(EventGroup, 0x50, pdFALSE, pdTRUE, portMAX_DELAY);
			xEventGroupClearBits(EventGroup, 0x10);
			break;
		case ('G'+'-'):
			exm.G_sign_value = 0;
			exm.G_sign_value = exm.current_instruction[1] + (exm.current_instruction[2]<<8) + (exm.current_instruction[2]<<16);
			if(exm.position == (0-exm.G_sign_value)) break;
			else if(exm.position > (0-exm.G_sign_value)){
				GPIOB->ODR |= GPIO_ODR_OD9; //counterclockwise
			}else{
				GPIOB->ODR &= ~GPIO_ODR_OD9; //clockwise
			}
//			  HAL_TIM_Base_Start_IT(&htim4);
//			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			if(exm.start_speed_value == 0) break;
			start_motor();
			xEventGroupWaitBits(EventGroup, 0x50, pdFALSE, pdTRUE, portMAX_DELAY);
			xEventGroupClearBits(EventGroup, 0x10);
			break;
		default:
			break;
	}
	  vTaskDelay(200);
  }
  /* USER CODE END execution_from_memory */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) {
		exm.step_counter_second_part++;
	}

	if (htim->Instance == TIM4) {
		if((xEventGroupGetBitsFromISR(EventGroup) & (1<<7)) != 0x80) { // if  Programming mode is OFF
			switch ((GPIOB->ODR & (GPIO_ODR_OD9))) {
				case (1<<9): //counterclockwise
					exm.position--;
				if(exm.position == 0-((int16_t)exm.G_sign_value)){
				  HAL_TIM_Base_Stop_IT(&htim4);
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
				xEventGroupSetBitsFromISR(EventGroup, 0x10, pdTRUE);
				}
					break;
				case (0<<9): //clockwise
					exm.position++;
					if(exm.position == exm.G_sign_value){
					  HAL_TIM_Base_Stop_IT(&htim4);
					HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
					xEventGroupSetBitsFromISR(EventGroup, 0x10, pdTRUE);
					}
					break;
				default:
					break;
			}
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
