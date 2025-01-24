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
#include <stdlib.h>
#include "dht22.h"
#include "I2C_LCD.h"
#include "stdio.h"
#include "hcsr04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t earliest_deadline;
    uint32_t end_cycle;
    uint32_t deadline;
    uint32_t period;
} TaskState;

TaskState tasks[] = {
    {0,0, 50, 100},  // Distance task
    {0,0, 100, 200},  // Temp task
    {0,0, 150, 300},  // Humidity task
};

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

UART_HandleTypeDef huart3;

/* Definitions for Get_Distance */
osThreadId_t Get_DistanceHandle;
const osThreadAttr_t Get_Distance_attributes = {
  .name = "Get_Distance",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Get_Temp */
osThreadId_t Get_TempHandle;
const osThreadAttr_t Get_Temp_attributes = {
  .name = "Get_Temp",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Get_Humidity */
osThreadId_t Get_HumidityHandle;
const osThreadAttr_t Get_Humidity_attributes = {
  .name = "Get_Humidity",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD_UART */
osThreadId_t LCD_UARTHandle;
const osThreadAttr_t LCD_UART_attributes = {
  .name = "LCD_UART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LCDQueue */
osMessageQueueId_t LCDQueueHandle;
const osMessageQueueAttr_t LCDQueue_attributes = {
  .name = "LCDQueue"
};
/* Definitions for EDF_Scheduler */
osTimerId_t EDF_SchedulerHandle;
const osTimerAttr_t EDF_Scheduler_attributes = {
  .name = "EDF_Scheduler"
};
/* Definitions for UARTMutex */
osMutexId_t UARTMutexHandle;
const osMutexAttr_t UARTMutex_attributes = {
  .name = "UARTMutex"
};
/* Definitions for LCDMutex */
osMutexId_t LCDMutexHandle;
const osMutexAttr_t LCDMutex_attributes = {
  .name = "LCDMutex"
};
/* USER CODE BEGIN PV */
int NUM_TASKS=3;
char buf[32]; // 1 dòng LCD chỉ có 16 ký tự
float T,H;

uint8_t RX_data,index_buf = 0;
int RX_done=0;
#define RX_BUFFER_SIZE 20
uint8_t rxBuffer[20];  // Bộ đệm nhận dữ liệu
uint8_t rxData;                    // Lưu dữ liệu từng byte

int task_id;
uint32_t deadline, period;

//int print_one_yet=0;

char received_char;
volatile bool system_running = false;  // Trạng thái hệ thống

#define FLAG_TASK_0 0x01
#define FLAG_TASK_1 0x02
#define FLAG_TASK_2 0x04

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void Function_Get_Distance(void *argument);
void Function_Get_Temp(void *argument);
void Function_Get_Humidity(void *argument);
void Function_LCD_UART(void *argument);
void CallbackEDF_Scheduler(void *argument);

/* USER CODE BEGIN PFP */
void Deadline_Init(){
	  uint32_t current_time = osKernelGetTickCount();  // Get the current system time
	  tasks[0].earliest_deadline =current_time + tasks[0].deadline;
	  tasks[1].earliest_deadline =current_time +tasks[1].deadline;
	  tasks[2].earliest_deadline =current_time +tasks[2].deadline;
	  tasks[0].end_cycle = current_time+tasks[0].period;
	  tasks[1].end_cycle = current_time+tasks[1].period;
	  tasks[2].end_cycle = current_time+tasks[2].period;
}
void EDF_Scheduler(void) {
    // In thông tin các deadline hiện tại để theo dõi
	uint32_t current_time = osKernelGetTickCount(); // Lấy thời gian hiện tại
//    printf("%lu:::%lu ----%lu----%lu\n",current_time, tasks[0].earliest_deadline, tasks[1].earliest_deadline, tasks[2].earliest_deadline);



    // Cập nhật lại deadline dựa trên chu kỳ cố định
    for (int i = 0; i < NUM_TASKS; i++) {
        if (current_time > tasks[i].end_cycle) {
            // Cập nhật deadline và chuyển sang chu kỳ kế tiếp
            tasks[i].earliest_deadline = tasks[i].end_cycle + tasks[i].deadline;
            tasks[i].end_cycle += tasks[i].period;
        }
    }

    // Tìm tác vụ có deadline sớm nhất
    int earliest_task = 0;
    for (int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].earliest_deadline < tasks[earliest_task].earliest_deadline) {
            earliest_task = i;
        }
    }
    if(tasks[earliest_task].earliest_deadline<tasks[earliest_task].end_cycle){
    	// Ghi nhật ký nhiệm vụ được ưu tiên (cho mục đích kiểm tra)
    	    printf("Task %d ->1 (Deadline: %lu)\n", earliest_task, tasks[earliest_task].earliest_deadline);


    	    osThreadFlagsSet(Get_DistanceHandle, earliest_task == 0 ? FLAG_TASK_0 : 0);
    	    osThreadFlagsSet(Get_TempHandle, earliest_task == 1 ? FLAG_TASK_1 : 0);
    	    osThreadFlagsSet(Get_HumidityHandle, earliest_task == 2 ? FLAG_TASK_2 : 0);
    	//
    	//    // Thiết lập ưu tiên cho các tác vụ
    	//    osThreadSetPriority(Get_DistanceHandle, earliest_task == 0 ? osPriorityNormal1 : osPriorityNormal);
    	//    osThreadSetPriority(Get_TempHandle, earliest_task == 1 ? osPriorityNormal1 : osPriorityNormal);
    	//    osThreadSetPriority(Get_HumidityHandle, earliest_task == 2 ? osPriorityNormal1 : osPriorityNormal);

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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  printf("start\n");
  printf("\"start.\"->start\n");

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HCSR04_Init(&htim1);
  dht22_init();
  I2C_LCD_Init(MyI2C_LCD);

  I2C_LCD_Clear(MyI2C_LCD);
  sprintf(buf,"\"start.\"->start");
  I2C_LCD_WriteString(MyI2C_LCD, buf);

//  HAL_UART_Receive_IT(&huart3, (uint8_t*)&received_char, 1);

  HAL_UART_Receive_IT(&huart3, &RX_data, 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of UARTMutex */
  UARTMutexHandle = osMutexNew(&UARTMutex_attributes);

  /* creation of LCDMutex */
  LCDMutexHandle = osMutexNew(&LCDMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of EDF_Scheduler */
  EDF_SchedulerHandle = osTimerNew(CallbackEDF_Scheduler, osTimerPeriodic, NULL, &EDF_Scheduler_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* Start the EDF Scheduler timer */
  if (osTimerStart(EDF_SchedulerHandle, 30) != osOK) {
      printf("Failed to start EDF Scheduler timer\n");
  }
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
  /* USER CODE END TIM3_Init 2 */

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
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    HAL_UART_Receive_IT(&huart3, (uint8_t*)&received_char, 1);
//
//    if (received_char == 's') {  // Lệnh 'S' để bắt đầu hệ thống
//        system_running = true;
////        if (osTimerStart(EDF_SchedulerHandle, 30) != osOK) {
////            printf("Failed to start EDF Scheduler timer\n");
////        }
//        Deadline_Init();
//        I2C_LCD_Clear(MyI2C_LCD);
//        LCD_UARTHandle = osThreadNew(Function_LCD_UART, NULL, &LCD_UART_attributes);
//        printf("System Started\n");
//    }
//    else if (received_char == 'p') {  // Lệnh 'P' để dừng hệ thống
////    	osTimerStop(EDF_SchedulerHandle);
//        system_running = false;
//        osThreadTerminate(LCD_UARTHandle);
//        I2C_LCD_Clear(MyI2C_LCD);
//		sprintf(buf,"Send s to start");
//		I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
//		I2C_LCD_WriteString(MyI2C_LCD, buf);
//        printf("System Stopped\n");
//    }
//}
// Hàm xử lý khi UART nhận dữ liệu xong

// Hàm phân tích lệnh và thay đổi thông số task
//void process_uart_command(const char *command) {
//    uint8_t task_id;
//    uint32_t deadline, period;
//
//    // Lệnh "set <task_id> <deadline> <period>"
//    if (sscanf(command, "set %hhu %lu %lu", &task_id, &deadline, &period) == 3) {
//        if (task_id < NUM_TASKS) {
//            tasks[task_id].deadline = deadline;
//            tasks[task_id].period = period;
//            printf("Task %d updated: Deadline=%lu, Period=%lu\n", task_id, deadline, period);
//        } else {
//            printf("Error: Invalid task ID. Use task_id between 0 and %d.\n", NUM_TASKS - 1);
//        }
//    }
//    // Lệnh "start" để khởi động hệ thống
//    else if (strcmp(command, "start") == 0) {
//        if (!system_running) {
//            system_running = true;
//            Deadline_Init();
//            I2C_LCD_Clear(MyI2C_LCD);
//            LCD_UARTHandle = osThreadNew(Function_LCD_UART, NULL, &LCD_UART_attributes);
//            printf("System started successfully.\n");
//        } else {
//            printf("System is already running.\n");
//        }
//    }
//    // Lệnh "stop" để dừng hệ thống
//    else if (strcmp(command, "stop") == 0) {
//        if (system_running) {
//            system_running = false;
//            osThreadTerminate(LCD_UARTHandle);
//            I2C_LCD_Clear(MyI2C_LCD);
//            sprintf(buf, "Send s to start");
//            I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
//            I2C_LCD_WriteString(MyI2C_LCD, buf);
//            printf("System stopped successfully.\n");
//        } else {
//            printf("System is not running.\n");
//        }
//    }
//    // Lệnh không hợp lệ
//    else {
//        printf("Error: Invalid command. Available commands:\n");
//        printf("  set <task_id> <deadline> <period>\n");
//        printf("  start\n");
//        printf("  stop\n");
//    }
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == USART3) { // Kiểm tra đúng UART3
//        if (uart_buffer[uart_buffer_index] == '\n' || uart_buffer[uart_buffer_index] == '\r') {
//            // Kết thúc lệnh -> xử lý lệnh
//            uart_buffer[uart_buffer_index] = '\0'; // Đảm bảo chuỗi kết thúc
//            process_uart_command(uart_buffer);    // Gọi hàm xử lý lệnh
//
//            // Reset bộ đệm để nhận lệnh mới
//            uart_buffer_index = 0;
//        } else {
//            // Tiếp tục lưu vào bộ đệm
//            uart_buffer_index++;
//            if (uart_buffer_index >= UART_BUFFER_SIZE) {
//                uart_buffer_index = 0; // Tránh tràn bộ đệm
//                printf("Error: UART buffer overflow. Command too long.\n");
//            }
//        }
//
//        // Bắt đầu nhận ký tự tiếp theo
//        HAL_UART_Receive_IT(huart, (uint8_t*)&uart_buffer[uart_buffer_index], 1);
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
				        system_running = true;
				//        if (osTimerStart(EDF_SchedulerHandle, 30) != osOK) {
				//            printf("Failed to start EDF Scheduler timer\n");
				//        }
				        Deadline_Init();
				        I2C_LCD_Clear(MyI2C_LCD);
				        LCD_UARTHandle = osThreadNew(Function_LCD_UART, NULL, &LCD_UART_attributes);
				        printf("System Started\n");
			}
			else if(strcmp(buf,"stop")==0){
				//    	osTimerStop(EDF_SchedulerHandle);
				        system_running = false;
				        osThreadTerminate(LCD_UARTHandle);
				        I2C_LCD_Clear(MyI2C_LCD);
						sprintf(buf,"\"start.\"->start");
						I2C_LCD_WriteString(MyI2C_LCD, buf);
				        printf("System Stopped\n");
			}
			else if(sscanf(buf, "set %d %lu %lu", &task_id, &deadline, &period)==3){
				tasks[task_id].deadline=deadline;
				tasks[task_id].period=period;
				printf("Setting done\n");
			}
		}
		HAL_UART_Receive_IT(&huart3, &RX_data, 1);
	}
}


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
	uint16_t id=1;
	while(1){
		osThreadFlagsWait(FLAG_TASK_0, osFlagsWaitAny, osWaitForever);

//		printf("test_d\n");
        if (osMutexAcquire(UARTMutexHandle, osWaitForever) == osOK) {
            // Thực hiện truy cập tài nguyên được bảo vệ
//			printf("test_d\n");


			if (hc04_state == HCSR04_IDLE_STATE) {
				HCSR04_Start();
			}
			int retry;
			retry = HCSR04_Handle();
			if(retry){
				HCSR04_Start();
				HCSR04_Handle();
			}
			tasks[0].earliest_deadline=tasks[0].end_cycle+tasks[0].deadline;
			// Send the result to the LCD queue

			osMessageQueuePut(LCDQueueHandle, &id, 0, osWaitForever);
			osMutexRelease(UARTMutexHandle);
        }
//		osDelay(tasks[0].period);
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Function_Get_Temp */
/* USER CODE END Header_Function_Get_Temp */
void Function_Get_Temp(void *argument)
{
  /* USER CODE BEGIN Function_Get_Temp */
	uint16_t id=2;
	while(1){
		osThreadFlagsWait(FLAG_TASK_1, osFlagsWaitAny, osWaitForever);

//		printf("test_t\n");
        if (osMutexAcquire(UARTMutexHandle, osWaitForever) == osOK) {
            // Thực hiện truy cập tài nguyên được bảo vệ
//        	printf("test_t\n");


		DHT22_Get_Temp(&T);


		// Send the result to the LCD queue
		tasks[1].earliest_deadline=tasks[1].end_cycle+tasks[1].deadline;
		osMessageQueuePut(LCDQueueHandle, &id, 0, osWaitForever);
		osMutexRelease(UARTMutexHandle);
        }
//		osDelay(tasks[1].period);
	}

  /* USER CODE END Function_Get_Temp */
}

/* USER CODE BEGIN Header_Function_Get_Humidity */
/* USER CODE END Header_Function_Get_Humidity */
void Function_Get_Humidity(void *argument)
{
  /* USER CODE BEGIN Function_Get_Humidity */
	uint16_t id=3;
	while(1){
		osThreadFlagsWait(FLAG_TASK_2, osFlagsWaitAny, osWaitForever);
//		printf("test_h\n");
        if (osMutexAcquire(UARTMutexHandle, osWaitForever) == osOK) {
            // Thực hiện truy cập tài nguyên được bảo vệ
//			printf("test_h\n");
			DHT22_Get_Humidity(&H);

			// Send the result to the LCD queue
			tasks[2].earliest_deadline=tasks[2].end_cycle+tasks[2].deadline;
			osMessageQueuePut(LCDQueueHandle, &id, 0, osWaitForever);
			osMutexRelease(UARTMutexHandle);
        }
//		osDelay(tasks[2].period);
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
        	printf("LCD-%d: ",received_id);

        	if (osMutexAcquire(LCDMutexHandle, osWaitForever) == osOK) {
      	  // Kiểm tra giá trị dữ liệu và thực hiện hành động tương ứng
      	  switch (received_id) {
      		  case 1:
//      			printf("LCD111\n");
					sprintf(buf,"D = %.1f cm", hcsr04_distance);
					I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
					I2C_LCD_WriteString(MyI2C_LCD, buf);
					printf("D = %.1f\r\n",hcsr04_distance);
					break;

      		  case 2:
//      			printf("LCD222\n");
					sprintf(buf,"T=%.1f", T);
					I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
					I2C_LCD_WriteString(MyI2C_LCD, buf);
					printf("T = %.1f\r\n",T);
					break;

      		  case 3:
//      			printf("LCD333\n");
					sprintf(buf,"H=%.1f", H);
					I2C_LCD_SetCursor(MyI2C_LCD, 7, 1);
					I2C_LCD_WriteString(MyI2C_LCD, buf);
					printf("H = %.1f\r\n",H);
					break;
      	  	  }
      	  	  osMutexRelease(LCDMutexHandle);
        	}
        }
    }
  /* USER CODE END Function_LCD_UART */
}

/* CallbackEDF_Scheduler function */
void CallbackEDF_Scheduler(void *argument)
{
  /* USER CODE BEGIN CallbackEDF_Scheduler */
//	if (osMutexAcquire(UARTMutexHandle, osWaitForever) == osOK) {
//		printf("callback\n");
	if (system_running){
		EDF_Scheduler();
	}
//	else{
////		if(!print_one_yet){
////			printf("call");
////			I2C_LCD_Clear(MyI2C_LCD);
////			I2C_LCD_WriteString(MyI2C_LCD, buf);
////			print_one_yet=1;
////		}
//	}

//		osMutexRelease(UARTMutexHandle);
//	}
  /* USER CODE END CallbackEDF_Scheduler */
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
