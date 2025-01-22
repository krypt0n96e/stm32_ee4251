/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "dht22.h"
#include "I2C_LCD.h"
#include "stdio.h"
#include "hcsr04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MyI2C_LCD I2C_LCD_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* Definitions for Get_Distance */
osThreadId_t Get_DistanceHandle;
const osThreadAttr_t Get_Distance_attributes = {
  .name = "Get_Distance",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Get_Temp */
osThreadId_t Get_TempHandle;
const osThreadAttr_t Get_Temp_attributes = {
  .name = "Get_Temp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Get_Humidity */
osThreadId_t Get_HumidityHandle;
const osThreadAttr_t Get_Humidity_attributes = {
  .name = "Get_Humidity",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_UART */
osThreadId_t LCD_UARTHandle;
const osThreadAttr_t LCD_UART_attributes = {
  .name = "LCD_UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LCDQueue */
osMessageQueueId_t LCDQueueHandle;
const osMessageQueueAttr_t LCDQueue_attributes = {
  .name = "LCDQueue"
};
/* USER CODE BEGIN PV */
char buf1[16],buf2[16]; // 1 dòng LCD chỉ có 16 ký tự
float T,H;
int T_Distance = 200, T_Temp = 400, T_Humidity = 600;  //Chu kỳ cho từng task
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void Function_Get_Distance(void *argument);
void Function_Get_Temp(void *argument);
void Function_Get_Humidity(void *argument);
void Function_LCD_UART(void *argument);

/* USER CODE BEGIN PFP */
void EDF_Scheduler(void) {
    static uint32_t last_exec_time_distance = 0;
    static uint32_t last_exec_time_temp = 0;
    static uint32_t last_exec_time_humidity = 0;
    uint32_t current_time = HAL_GetTick();  // Get the current system time

    // Task periods for EDF
    const uint32_t Get_Distance_period = T_Distance;  // Period (ms)
    const uint32_t Get_Temp_period = T_Temp;      // Period (ms)
    const uint32_t Get_Humidity_period = Get_Temp_period;  // Period (ms)

    // EDF Scheduling logic
    // Ví dụ thay đổi mức ưu tiên trong EDF scheduler
    if (current_time - last_exec_time_distance >= Get_Distance_period) {
    	printf("d->1\n");
        osThreadFlagsSet(Get_DistanceHandle, 0x01);
        osThreadSetPriority(Get_DistanceHandle, osPriorityHigh);  // Thay đổi mức ưu tiên của task
        osThreadSetPriority(Get_HumidityHandle, osPriorityNormal);
        osThreadSetPriority(Get_TempHandle, osPriorityNormal);
        last_exec_time_distance = current_time;
    }
    if (current_time - last_exec_time_temp >= Get_Temp_period) {
    	printf("t->1\n");
        osThreadFlagsSet(Get_TempHandle, 0x01);
        osThreadSetPriority(Get_TempHandle, osPriorityHigh);  // Thay đổi mức ưu tiên của task
        osThreadSetPriority(Get_DistanceHandle, osPriorityNormal);
        osThreadSetPriority(Get_HumidityHandle, osPriorityNormal);
        last_exec_time_temp = current_time;
    }
    if (current_time - last_exec_time_humidity >= Get_Humidity_period) {
    	printf("h->1\n");
        osThreadFlagsSet(Get_HumidityHandle, 0x01);
        osThreadSetPriority(Get_HumidityHandle, osPriorityHigh);  // Thay đổi mức ưu tiên của task
        osThreadSetPriority(Get_DistanceHandle, osPriorityNormal);
        osThreadSetPriority(Get_TempHandle, osPriorityNormal);
        last_exec_time_humidity = current_time;
    }

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);  // Transmit char via UART
    return ch;
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  printf("start\n");
  HCSR04_Init(&htim1);
  dht22_init();
  I2C_LCD_Init(MyI2C_LCD);
  EDF_Scheduler();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LCDQueue */
  LCDQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &LCDQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Get_Distance */
  Get_DistanceHandle = osThreadNew(Function_Get_Distance, NULL, &Get_Distance_attributes);

  /* creation of Get_Temp */
  Get_TempHandle = osThreadNew(Function_Get_Temp, NULL, &Get_Temp_attributes);

  /* creation of Get_Humidity */
  Get_HumidityHandle = osThreadNew(Function_Get_Humidity, NULL, &Get_Humidity_attributes);

  /* creation of LCD_UART */
  LCD_UARTHandle = osThreadNew(Function_LCD_UART, NULL, &LCD_UART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 29999;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
	__HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
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
  HAL_TIM_Base_Start(&htim3);
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Base_Start_IT(&htim4);  // Kích hoạt ngắt Timer 4
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Function_Get_Distance */
/**
  * @brief  Function implementing the Get_Distance thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Function_Get_Distance */
void Function_Get_Distance(void *argument)
{
  /* USER CODE BEGIN 5 */
	while(1){
		osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);  // Wait for the EDF scheduler signal
		printf("test_d\n");

		uint16_t id=1;
		if (hc04_state == HCSR04_IDLE_STATE) {
			HCSR04_Start();
			HCSR04_Handle();
			// Send the result to the LCD queue
			osMessageQueuePut(LCDQueueHandle, &id, 0, osWaitForever);
		}
		EDF_Scheduler();
		osDelay(T_Distance);
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Function_Get_Temp */
/* USER CODE END Header_Function_Get_Temp */
void Function_Get_Temp(void *argument)
{
  /* USER CODE BEGIN Function_Get_Temp */
	while(1){
		osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);  // Wait for the EDF scheduler signal
		printf("test_t\n");

		uint16_t id=2;
		DHT22_Get_Temp(&T);


		// Send the result to the LCD queue
		osMessageQueuePut(LCDQueueHandle, &id, 0, osWaitForever);
		EDF_Scheduler();
		osDelay(T_Temp);
	}

  /* USER CODE END Function_Get_Temp */
}

/* USER CODE BEGIN Header_Function_Get_Humidity */
/* USER CODE END Header_Function_Get_Humidity */
void Function_Get_Humidity(void *argument)
{
  /* USER CODE BEGIN Function_Get_Humidity */

	while(1){
		osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);  // Wait for the EDF scheduler signal
		printf("test_h\n");
		DHT22_Get_Humidity(&H);
		uint16_t id=3;
		// Send the result to the LCD queue
		osMessageQueuePut(LCDQueueHandle, &id, 0, osWaitForever);
		EDF_Scheduler();
		osDelay(T_Humidity);
	}

  /* USER CODE END Function_Get_Humidity */
}

/* USER CODE BEGIN Header_Function_LCD_UART */
/* USER CODE END Header_Function_LCD_UART */
void Function_LCD_UART(void *argument)
{
  /* USER CODE BEGIN Function_LCD_UART */
	uint16_t received_id;

    while (1) {
        // Check if data is available in the queue

        if (osMessageQueueGet(LCDQueueHandle, &received_id, NULL, osWaitForever) == osOK) {
        	printf("LCD-%d\n",received_id);

      	  // Kiểm tra giá trị dữ liệu và thực hiện hành động tương ứng
      	  switch (received_id) {
      		  case 1:
      			printf("LCD111\n");
					sprintf(buf1,"D = %.1f cm", hcsr04_distance);
					I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
					I2C_LCD_WriteString(MyI2C_LCD, buf1);
					printf("D = %.1f\r\n",hcsr04_distance);
					break;

      		  case 2:
      			printf("LCD222\n");
					sprintf(buf2,"T=%.1f", T);
					I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
					I2C_LCD_WriteString(MyI2C_LCD, buf2);
					printf("T = %.1f\r\n",T);
					break;

      		  case 3:
      			printf("LCD333\n");
					sprintf(buf2,"H=%.1f", H);
					I2C_LCD_SetCursor(MyI2C_LCD, 7, 1);
					I2C_LCD_WriteString(MyI2C_LCD, buf2);
					printf("H = %.1f\r\n",H);
					break;
      	  }
        }
        osDelay(1);


    }
  /* USER CODE END Function_LCD_UART */
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
  if (htim->Instance == TIM4) {
//      EDF_Scheduler(); // Gọi hàm EDF Scheduler mỗi khi Timer 4 tạo ngắt
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
