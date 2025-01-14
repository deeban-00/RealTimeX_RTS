/* USER CODE BEGIN Header */
/**
  ******************************************************************************  
  * @file           : main.c
  * @brief          : Main program body for Lab Evaluation 2 (PCP & EDF)
  ******************************************************************************  
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usart.h" // UART config
#include "gpio.h"  // GPIO config

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

TIM_HandleTypeDef htim1;
osThreadId_t ledTaskHandle;
osThreadId_t distanceTaskHandle;
osMutexId_t uartMutexHandle;

/* USER CODE BEGIN PV */
// Ultrasonic sensor trigger and echo pins
#define TRIG_PIN GPIO_PIN_9
#define ECHO_PIN GPIO_PIN_8
#define GPIO_PORT GPIOA
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
// Function Prototypes
void StartLEDBlinkTask(void *argument);
void StartDistanceCalcTask(void *argument);
void AdjustPriorities(void);
int MeasureDistance(void);
/* USER CODE BEGIN PFP */

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
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  // Create Mutex for UART
  osMutexAttr_t uartMutexAttributes = { "uartMutex" };
  uartMutexHandle = osMutexNew(&uartMutexAttributes);

  // Create Tasks
  osThreadAttr_t ledTaskAttr = { .name = "ledTask", .priority = osPriorityNormal };
  osThreadAttr_t distanceTaskAttr = { .name = "distanceTask", .priority = osPriorityNormal };
  
  ledTaskHandle = osThreadNew(StartLEDBlinkTask, NULL, &ledTaskAttr);
  distanceTaskHandle = osThreadNew(StartDistanceCalcTask, NULL, &distanceTaskAttr);

  // Start Scheduler
  osKernelStart();
  
  while (1) {
    // Main infinite loop
  }
}

/**
  * @brief  LED Blinking Task for PCP & EDF
  * @retval None
  */
void StartLEDBlinkTask(void *argument)
{
  for (;;)
  {
    osMutexAcquire(uartMutexHandle, osWaitForever);
    HAL_GPIO_TogglePin(GPIO_PORT, GPIO_PIN_13); // Toggle LED
    osMutexRelease(uartMutexHandle);
    
    AdjustPriorities(); // Simulate EDF priority check
    osDelay(1000); // 1-second delay
  }
}

/**
  * @brief  Distance Calculation Task for PCP & EDF
  * @retval None
  */
void StartDistanceCalcTask(void *argument)
{
  for (;;)
  {
    osMutexAcquire(uartMutexHandle, osWaitForever);
    int distance = MeasureDistance();  // Measure distance using ultrasonic sensor
    osMutexRelease(uartMutexHandle);
    
    AdjustPriorities(); // Simulate EDF priority check
    osDelay(500); // Adjust delay for task frequency
  }
}

/**
  * @brief  Measure distance using ultrasonic sensor
  * @retval distance in cm
  */
int MeasureDistance(void)
{
  uint32_t startTime = 0, stopTime = 0;
  int distance = 0;

  // Trigger Ultrasonic Sensor
  HAL_GPIO_WritePin(GPIO_PORT, TRIG_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIO_PORT, TRIG_PIN, GPIO_PIN_RESET);

  // Wait for echo start and stop
  while (HAL_GPIO_ReadPin(GPIO_PORT, ECHO_PIN) == GPIO_PIN_RESET);
  startTime = __HAL_TIM_GET_COUNTER(&htim1);

  while (HAL_GPIO_ReadPin(GPIO_PORT, ECHO_PIN) == GPIO_PIN_SET);
  stopTime = __HAL_TIM_GET_COUNTER(&htim1);

  // Calculate distance
  distance = ((stopTime - startTime) * 0.0343) / 2;
  return distance;
}

/**
  * @brief  Adjust task priorities for EDF scheduling
  * @retval None
  */
void AdjustPriorities(void)
{
  // Dummy deadlines for tasks in ms
  const int ledTaskDeadline = 2000;   // 2 seconds
  const int distanceTaskDeadline = 1000; // 1 second

  // Adjust task priorities based on deadlines
  if (ledTaskDeadline < distanceTaskDeadline) {
    osThreadSetPriority(ledTaskHandle, osPriorityAboveNormal);
    osThreadSetPriority(distanceTaskHandle, osPriorityNormal);
  } else {
    osThreadSetPriority(ledTaskHandle, osPriorityNormal);
    osThreadSetPriority(distanceTaskHandle, osPriorityAboveNormal);
  }
}

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
