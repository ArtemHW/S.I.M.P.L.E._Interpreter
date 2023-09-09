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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

osThreadId ProgramingModeHandle;
osThreadId InterpreterHandle;
osThreadId UART_comunicationHandle;
osThreadId ExecutionFromMemoryHandle;
/* USER CODE BEGIN PV */
uint8_t counter_for_steps = 0;
QueueHandle_t uart_queue_rx;
//QueueHandle_t instruction_S_value;

struct exm_type{
	char execution_memory[119]; //Program storage capability of GS-C200S
	uint8_t memory_pointer;
	uint8_t start_of_instruction;
	uint8_t size_of_instruction;
	uint16_t start_speed_value;

	void (*function_pointer_S)(void);
}exm;


EventGroupHandle_t EventGroup;
/* Event Group description
 *  0x80 Programming mode for GS-200S
 *	0x40 Execution mode
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
void programing_mode(void const * argument);
void interpreter(void const * argument);
void uart_comunication(void const * argument);
void execution_from_memory(void const * argument);

/* USER CODE BEGIN PFP */
void uart1_rx_callback();
void function_of_S();
void write_to_exm(uint8_t* start_of_data, uint8_t size_of_data);
void erase_exm();
void perform_instruction_from_exm();
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
	counter_for_steps = 0;

	for(int i = 0; i < sizeof(exm.execution_memory); i++){
		exm.execution_memory[i] = 0;
	}
	exm.memory_pointer = 0;
	exm.size_of_instruction = 0;
	exm.start_of_instruction = 0;
	exm.function_pointer_S = function_of_S;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  uint16_t base = 2000;
  TIM3->ARR = base;
  TIM3->CCR1 = (uint16_t) base*0.98;
  TIM3->CCR2 = (uint16_t) base*0.98;
  TIM3->CCR3 = (uint16_t) base*0.98;
  TIM3->CCR4 = (uint16_t) base*0.98;

  USART1->CR1 |= USART_CR1_RXNEIE;
  USART1->CR1 |= USART_CR1_TE;
  USART1->CR1 |= USART_CR1_RE;
  USART1->CR1 |= USART_CR1_UE; //USART enable

  GPIOC->ODR &= ~GPIO_ODR_OD9; //Enable stepper driver
  HAL_TIM_Base_Start_IT(&htim10);

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
  uart_queue_rx = xQueueCreate(10, 1);
  //instruction_S_value = xQueueCreate(5, 2);

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
  osThreadDef(ExecutionFromMemory, execution_from_memory, osPriorityNormal, 0, 500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  sConfigOC.Pulse = 980;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENA_stepper_GPIO_Port, ENA_stepper_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_stepper_GPIO_Port, DIR_stepper_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENA_stepper_Pin */
  GPIO_InitStruct.Pin = ENA_stepper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENA_stepper_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_stepper_Pin */
  GPIO_InitStruct.Pin = DIR_stepper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIR_stepper_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void uart1_rx_callback(void)
{
	xQueueSendToBackFromISR(uart_queue_rx, (void*)&(USART1->DR), pdTRUE);
	__asm__ volatile("NOP");

}

void write_to_exm(uint8_t* start_of_data, uint8_t size_of_data){
	for(int i = 0; i < size_of_data; i++){
		exm.execution_memory[exm.memory_pointer] = start_of_data[i];
		exm.memory_pointer++;
	}
}

void erase_exm(){
	for(int i = 0; i < sizeof(exm.execution_memory); i++){
		exm.execution_memory[i] = 0;
	}
}

void perform_instruction_from_exm(){

}

void function_of_S(){
	uint32_t base = 1000000/exm.start_speed_value; // Speed of HCLK is 16MHz but prescaler for TIM3 is 15 (15+1) so you receive 1MHz clock for TIM3
	  TIM3->ARR = base;
	  TIM3->CCR1 = (uint16_t) base*0.99;
	  TIM3->CCR2 = (uint16_t) base*0.99;
	  TIM3->CCR3 = (uint16_t) base*0.99;
	  TIM3->CCR4 = (uint16_t) base*0.99;
	__asm__ volatile("NOP");
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
	  vTaskDelay(200);

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
					xEventGroupClearBits(EventGroup, 0x40);
					exm.memory_pointer = 0;
					erase_exm();
					xEventGroupSetBits(EventGroup, 0x80);
					break;
				case 'x':
					xEventGroupClearBits(EventGroup, 0x80);
					exm.memory_pointer = 0;
					xEventGroupSetBits(EventGroup, 0x40);
					break;
				default:
					break;
			}
			break;
	    case 'E':
	    	xEventGroupClearBits(EventGroup, 0x80);
	    	exm.memory_pointer = 0;
	    	xEventGroupSetBits(EventGroup, 0x40);
	    	break;
		case 'S':
			if((xEventGroupGetBits(EventGroup) & (1<<7)) != 0x80) break; // if  Programming mode is OFF
			exm.start_speed_value = 0;
			char temp = 0;
			for(int i = 0; i < 4; i++){
				xQueueReceive(uart_queue_rx, &temp, 5);
				if((temp == 13) || (temp == 0) || (i == 3)) break;
				if(i == 3) break;
				if ((temp & (1<<7)) == 0x80){
					temp &= ~(1<<7);
			    }
				exm.start_speed_value = (exm.start_speed_value*10) + (temp - 48);
			}
			uint8_t data[4] = {'S', (uint8_t)(exm.start_speed_value), (uint8_t)((exm.start_speed_value>>8)), 0};
			write_to_exm(data, sizeof(data));
			//exm.
			__asm__ volatile("NOP");
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
	GPIOB->ODR |= GPIO_ODR_OD9;
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  /* Infinite loop */
  for(;;)
  {

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
	  perform_instruction_from_exm();


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

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3) {
		switch (counter_for_steps) {
			case 0:
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				break;
			case 1:
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				break;
			case 2:
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
				break;
			case 3:
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				break;
			default:
				break;
		}
		counter_for_steps == 3 ? counter_for_steps = 0 : counter_for_steps++;
  }
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
