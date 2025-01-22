/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* Definitions for Get_Distance */
osThreadId_t Get_DistanceHandle;
const osThreadAttr_t Get_Distance_attributes = {
  .name = "Get_Distance",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Get_Temp */
osThreadId_t Get_TempHandle;
const osThreadAttr_t Get_Temp_attributes = {
  .name = "Get_Temp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Get_Humidity */
osThreadId_t Get_HumidityHandle;
const osThreadAttr_t Get_Humidity_attributes = {
  .name = "Get_Humidity",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Control_On */
osThreadId_t Control_OnHandle;
const osThreadAttr_t Control_On_attributes = {
  .name = "Control_On",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for LCD_UART */
osThreadId_t LCD_UARTHandle;
const osThreadAttr_t LCD_UART_attributes = {
  .name = "LCD_UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Set_CDT */
osThreadId_t Set_CDTHandle;
const osThreadAttr_t Set_CDT_attributes = {
  .name = "Set_CDT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for Control_Off */
osThreadId_t Control_OffHandle;
const osThreadAttr_t Control_Off_attributes = {
  .name = "Control_Off",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for Queue_CDT */
osMessageQueueId_t Queue_CDTHandle;
const osMessageQueueAttr_t Queue_CDT_attributes = {
  .name = "Queue_CDT"
};
/* Definitions for Mutex_DHT22 */
osMutexId_t Mutex_DHT22Handle;
const osMutexAttr_t Mutex_DHT22_attributes = {
  .name = "Mutex_DHT22"
};
/* Definitions for Mutex_HCSR04 */
osMutexId_t Mutex_HCSR04Handle;
const osMutexAttr_t Mutex_HCSR04_attributes = {
  .name = "Mutex_HCSR04"
};
/* Definitions for Sem_Set_CDT */
osSemaphoreId_t Sem_Set_CDTHandle;
const osSemaphoreAttr_t Sem_Set_CDT_attributes = {
  .name = "Sem_Set_CDT"
};
/* Definitions for Sem_On */
osSemaphoreId_t Sem_OnHandle;
const osSemaphoreAttr_t Sem_On_attributes = {
  .name = "Sem_On"
};
/* Definitions for Sem_Off */
osSemaphoreId_t Sem_OffHandle;
const osSemaphoreAttr_t Sem_Off_attributes = {
  .name = "Sem_Off"
};
/* USER CODE BEGIN PV */
char buf1[16],buf2[16],buf[20]; // 1 dòng LCD chỉ có 16 ký tự
float T,H;
int time_offset = 100, i=0, interprut = 0;
uint8_t RX_data,index_buf = 0;
int start =0,RX_done=0;
uint32_t count=0;
#define RX_BUFFER_SIZE 20
uint8_t rxBuffer[20];  // Bộ đệm nhận dữ liệu
uint8_t rxData;                    // Lưu dữ liệu từng byte
int T_Distace = 200, T_Temp = 400, T_Humidity = 600;  //Chu kỳ cho từng task
int received_id; //id push vào hàng đợi
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void Funciton_Get_Distance(void *argument);
void Function_Get_Temp(void *argument);
void Function_Get_Humidity(void *argument);
void Function_Control_On(void *argument);
void Function_LCD_UART(void *argument);
void Function_Set_CDT(void *argument);
void Function_Control_Off(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef	__GNUC__
		#define PUTCHAR_PROTOTYPE int	__io_putchar(int ch)
#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
 PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch,1,0xFFFF);
	return ch;
}
 void SPTTS_Task1(void){
	 if (hc04_state == HCSR04_IDLE_STATE) {
	 	HCSR04_Start();
	 }
	 HCSR04_Handle();
	 sprintf(buf1,"D = %.1f cm", hcsr04_distance);
	 sprintf(buf2,"T=%.1f,H=%.1f", T,H);
	 I2C_LCD_Clear(MyI2C_LCD);
	 I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
	 I2C_LCD_WriteString(MyI2C_LCD, buf1);
	 I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
	 I2C_LCD_WriteString(MyI2C_LCD, buf2);
	 printf("D = %.1f\r\n",hcsr04_distance);
 }
 void SPTTS_Task2(void){
 	DHT22_Get_Humidity(&H);
 	sprintf(buf1,"D = %.1f cm", hcsr04_distance);
 	sprintf(buf2,"T=%.1f,H=%.1f", T,H);
 	I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
 	I2C_LCD_Clear(MyI2C_LCD);
 	I2C_LCD_WriteString(MyI2C_LCD, buf1);
 	I2C_LCD_SetCursor(MyI2C_LCD, 0, 11);
 	I2C_LCD_WriteString(MyI2C_LCD, buf2);
	printf("H = %.1f\r\n",H);
  }
 void SPTTS_Task3(void){
	 DHT22_Get_Temp(&T);
	 sprintf(buf1,"D = %.1f cm", hcsr04_distance);
	 sprintf(buf2,"T=%.1f,H=%.1f", T,H);
	 I2C_LCD_Clear(MyI2C_LCD);
	 I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
	 I2C_LCD_WriteString(MyI2C_LCD, buf1);
	 I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
	 I2C_LCD_WriteString(MyI2C_LCD, buf2);
	 printf("T = %.1f\r\n",T);
 }
 bool is_valid_format(const char *input, int num1, int num2, int num3, int num4) {
 			    // Sử dụng sscanf để kiểm tra định dạng
 			    int count = sscanf(input, "%d %d %d %d", &num1, &num2, &num3, &num4);

 			    // Nếu đúng định dạng "%d %d %d %d", sscanf sẽ trả về 4 (số giá trị đọc được)
 			    // Nếu có thêm ký tự dư thừa, count sẽ lớn hơn 4
 			    if ((num1 == 1 || num1==2 || num1 == 3)&& (count ==4)){
 			    	return 1;
 			    }
 			    else return 0;

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  // Bắt đầu Timer 2 ở chế độ ngắt
    //HAL_TIM_Base_Start_IT(&htim2);
  dht22_init();
  HCSR04_Init(&htim1);
  I2C_LCD_Init(MyI2C_LCD);
  HAL_UART_Receive_IT(&huart3, &RX_data, 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Mutex_DHT22 */
  Mutex_DHT22Handle = osMutexNew(&Mutex_DHT22_attributes);

  /* creation of Mutex_HCSR04 */
  Mutex_HCSR04Handle = osMutexNew(&Mutex_HCSR04_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem_Set_CDT */
  Sem_Set_CDTHandle = osSemaphoreNew(1, 1, &Sem_Set_CDT_attributes);

  /* creation of Sem_On */
  Sem_OnHandle = osSemaphoreNew(1, 1, &Sem_On_attributes);

  /* creation of Sem_Off */
  Sem_OffHandle = osSemaphoreNew(1, 1, &Sem_Off_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (2, sizeof(uint8_t), &myQueue01_attributes);

  /* creation of Queue_CDT */
  Queue_CDTHandle = osMessageQueueNew (4, sizeof(uint8_t), &Queue_CDT_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Get_Distance */
  Get_DistanceHandle = osThreadNew(Funciton_Get_Distance, NULL, &Get_Distance_attributes);

  /* creation of Get_Temp */
  Get_TempHandle = osThreadNew(Function_Get_Temp, NULL, &Get_Temp_attributes);

  /* creation of Get_Humidity */
  Get_HumidityHandle = osThreadNew(Function_Get_Humidity, NULL, &Get_Humidity_attributes);

  /* creation of Control_On */
  Control_OnHandle = osThreadNew(Function_Control_On, NULL, &Control_On_attributes);

  /* creation of LCD_UART */
  LCD_UARTHandle = osThreadNew(Function_LCD_UART, NULL, &LCD_UART_attributes);

  /* creation of Set_CDT */
  Set_CDTHandle = osThreadNew(Function_Set_CDT, NULL, &Set_CDT_attributes);

  /* creation of Control_Off */
  Control_OffHandle = osThreadNew(Function_Control_Off, NULL, &Control_Off_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == huart3.Instance) {
//    	printf("Da nhay vao ngat\r\n");
//    	osSemaphoreRelease(Sem_OnHandle);
//    	switch (RX_data){
//			case 48:
//				printf("Giai phong semaphor off\r\n");
//				start = 0;
//				break;
//			case 49:
//				printf("Giai phong semaphor on\r\n");
//				start = 1;
//				break;
//			default:
//				// Không hợp lệ, không thực hiện gì
//				break;
//    	 }
//    	HAL_UART_Receive_IT(&huart3, &RX_data, 1);
//    }
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == huart3.Instance) {
//        if (RX_data == 48) { // '0'
//            osThreadFlagsSet(Control_OffHandle, 0x01);
//            printf("ThreadFlag Off\r\n");// Gửi tín hiệu đến task Control_Off
//        } else if (RX_data == 49) { // '1'
//        	int id = 1;
//        	osMessageQueuePut(Queue_CDTHandle, &id, 0, 0);
//        }else if (RX_data == 50){
//        	int id = 2;
//        	osMessageQueuePut(Queue_CDTHandle, &id, 0, 0);
//        }
//
//        HAL_UART_Receive_IT(&huart3, &RX_data, 1); // Tiếp tục nhận UART
//    }
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	UNUSED(huart);
	if (huart->Instance == huart3.Instance) {
		if(RX_data !=46)
		{ //NULL ASCII
			rxBuffer[index_buf++] = RX_data;//Them du lieu vao buffer
		}
		else if(RX_data==46)
		{
			index_buf = 0; // xoa con tro du lieu
			sprintf (buf,rxBuffer);
			memset(rxBuffer,0,strlen(rxBuffer));
			if(strcmp(buf,"start")==0){
				int id = 1;
				osMessageQueuePut(Queue_CDTHandle, &id, 0, 0);
			}
			if(strcmp(buf,"stop")==0){
				osThreadFlagsSet(Control_OffHandle, 0x01);
			}
			if(strcmp(buf,"set")==0){
				int id = 2;
				osMessageQueuePut(Queue_CDTHandle, &id, 0, 0);
			}
		}
		HAL_UART_Receive_IT(&huart3, &RX_data, 1);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Funciton_Get_Distance */
/**
  * @brief  Function implementing the Get_Distance thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Funciton_Get_Distance */
void Funciton_Get_Distance(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	printf("@%lu Start task Distance\r\n", osKernelGetSysTimerCount());
	int id = 1;
	if (hc04_state == HCSR04_IDLE_STATE) {
		HCSR04_Start();
	}

	HCSR04_Handle();
	osMessageQueuePut(myQueue01Handle, &id, 0, 0);
	// Giải phóng quyền sử dụng tài nguyên
    osDelay(T_Distace );
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Function_Get_Temp */
/**
* @brief Function implementing the Get_Temp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Function_Get_Temp */
void Function_Get_Temp(void *argument)
{
  /* USER CODE BEGIN Function_Get_Temp */
	osPriority_t originalPriority = osThreadGetPriority(Get_TempHandle); // Lưu mức ưu tiên ban đầu
  /* Infinite loop */
  for(;;)
  {
	printf("@%lu Start task Temp\r\n", osKernelGetSysTimerCount());
	osThreadSetPriority(Get_TempHandle, osPriorityRealtime); // Tăng ưu tiên tạm thời
	int id = 3;
	DHT22_Get_Temp(&T);
	osMessageQueuePut(myQueue01Handle, &id, 0, 0);
	osThreadSetPriority(Get_TempHandle, originalPriority); // Khôi phục ưu tiên ban đầu
    osDelay(T_Temp);
  }
  /* USER CODE END Function_Get_Temp */
}

/* USER CODE BEGIN Header_Function_Get_Humidity */
/**
* @brief Function implementing the Get_Humidity thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Function_Get_Humidity */
void Function_Get_Humidity(void *argument)
{
  /* USER CODE BEGIN Function_Get_Humidity */
	osPriority_t originalPriority = osThreadGetPriority(Get_HumidityHandle); // Lưu mức ưu tiên ban đầu
  /* Infinite loop */
  for(;;)
  {
	printf("@%lu Start task Humi\r\n", osKernelGetSysTimerCount());
	osThreadSetPriority(Get_HumidityHandle, osPriorityRealtime); // Tăng ưu tiên tạm thời
	int id =2;
	DHT22_Get_Humidity(&H);
	osMessageQueuePut(myQueue01Handle, &id, 0, 0); // Push id vào hàng đợi
	osThreadSetPriority(Get_HumidityHandle, originalPriority); // Khôi phục ưu tiên ban đầu
    osDelay(T_Humidity);
  }
  /* USER CODE END Function_Get_Humidity */
}

/* USER CODE BEGIN Header_Function_Control_On */
/**
* @brief Function implementing the Control_On thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Function_Control_On */
void Function_Control_On(void *argument)
{
  /* USER CODE BEGIN Function_Control_On */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever); // Chờ tín hiệu từ ISR
	  printf("System resumed.\r\n");
	  osThreadResume(Get_DistanceHandle);
	  osThreadResume(Get_HumidityHandle);
	  osThreadResume(Get_TempHandle);
  }
  /* USER CODE END Function_Control_On */
}

/* USER CODE BEGIN Header_Function_LCD_UART */
/**
* @brief Function implementing the LCD_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Function_LCD_UART */
void Function_LCD_UART(void *argument)
{
  /* USER CODE BEGIN Function_LCD_UART */
  /* Infinite loop */
  for(;;)
  {
// Chờ dữ liệu từ hàng đợi
	  osMessageQueueGet(myQueue01Handle,&received_id, NULL, osWaitForever);
	  // Kiểm tra giá trị dữ liệu và thực hiện hành động tương ứng
	  switch (received_id) {
		  case 1:
			  sprintf(buf1,"D = %.1f cm", hcsr04_distance);
			  I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
			  I2C_LCD_WriteString(MyI2C_LCD, buf1);
			  printf("D = %.1f\r\n",hcsr04_distance);
			  break;

		  case 2:
			sprintf(buf2,"T=%.1f,H=%.1f", T,H);
			I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
			I2C_LCD_WriteString(MyI2C_LCD, buf2);
			printf("H = %.1f\r\n",H);
			break;

		  case 3:
			 sprintf(buf2,"T=%.1f,H=%.1f", T,H);
			 I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
			 I2C_LCD_WriteString(MyI2C_LCD, buf2);
			 printf("T = %.1f\r\n",T);
			 break;
	  }
  }
  /* USER CODE END Function_LCD_UART */
}

/* USER CODE BEGIN Header_Function_Set_CDT */
/**
* @brief Function implementing the Set_CDT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Function_Set_CDT */
void Function_Set_CDT(void *argument)
{
  /* USER CODE BEGIN Function_Set_CDT */
  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(Queue_CDTHandle,&received_id, NULL, osWaitForever);
	 switch (received_id) {
		case 1:
				  printf("Resume System\r\n");
				  I2C_LCD_Clear(MyI2C_LCD);
				  osThreadResume(Get_DistanceHandle);
				  osThreadResume(Get_TempHandle);
				  osThreadResume(Get_HumidityHandle);
				  break;
		case 2:
			osThreadSuspend(Get_DistanceHandle);
			osThreadSuspend(Get_HumidityHandle);
			osThreadSuspend(Get_TempHandle);
			printf("Nhap tham so thoi gian theo dataframe: id_task C D T\r\n") ;
			printf("Luu y: C co 3 chu so, D&T co 4 chu so\r\n");
			int id,C,D,T;
			while (!RX_done){
				if(strlen(buf)>0){
					if(is_valid_format(buf,&id,&C,&D,&T)){
						break;
					}
					else{
						memset(buf,0,strlen(buf));
						printf("Hay nhap dung dinh dang tham so\r\n");
						HAL_UART_Receive_IT(&huart3, &RX_data, 1);
					}
				}
			}
			printf("Nhan duoc tham so hop le\r\n");
			switch (id){
			case 1:
				T_Distace = T;
				break;
			case 2:
				T_Humidity = T;
				break;
			case 3:
				T_Temp = T;
				break;
			}
			printf("Da set tham so thanh cong!, he thong se khoi dong lai sau 2s...\r\n");
			HAL_Delay(2000);
			osThreadResume(Get_DistanceHandle);
			osThreadResume(Get_HumidityHandle);
			osThreadResume(Get_TempHandle);
	 }
  /* USER CODE END Function_Set_CDT */
  }
}

/* USER CODE BEGIN Header_Function_Control_Off */
/**
* @brief Function implementing the Control_Off thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Function_Control_Off */
void Function_Control_Off(void *argument)
{
  /* USER CODE BEGIN Function_Control_Off */
	osThreadSuspend(Get_DistanceHandle);
	osThreadSuspend(Get_HumidityHandle);
	osThreadSuspend(Get_TempHandle);
	I2C_LCD_Clear(MyI2C_LCD);
	sprintf(buf,"   Send start");
	I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
	I2C_LCD_WriteString(MyI2C_LCD, buf);
	sprintf(buf," to run system");
	I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
	I2C_LCD_WriteString(MyI2C_LCD, buf);
	printf("Gui start qua UART de chay he thong\r\n");

  /* Infinite loop */
  for(;;)
  {
	 osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever); // Chờ tín hiệu từ ISR

	 osThreadSuspend(Get_DistanceHandle);
	 osThreadSuspend(Get_HumidityHandle);
	 osThreadSuspend(Get_TempHandle);
	 I2C_LCD_Clear(MyI2C_LCD);
	 printf("System paused.\r\n");
	 printf("Send start to resume system");
	 sprintf(buf,"   Send start");
	 I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
	 I2C_LCD_WriteString(MyI2C_LCD, buf);
	 sprintf(buf,"to resume system");
	 I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
	 I2C_LCD_WriteString(MyI2C_LCD, buf);
  }
  /* USER CODE END Function_Control_Off */
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
