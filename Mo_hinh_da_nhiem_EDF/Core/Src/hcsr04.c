#include "hcsr04.h"

// Định nghĩa các biến toàn cục
HCSR04_State hc04_state = HCSR04_IDLE_STATE;
TIM_HandleTypeDef *htim = NULL;
float hcsr04_distance = -1;

// Hàm tạo xung tín hiệu
static void pulseGPIO(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// Khởi tạo cảm biến HCSR04
void HCSR04_Init(TIM_HandleTypeDef *timer) {

    htim = timer;  // Gán con trỏ timer
}

// Hàm delay micro giây
void Delay_us(uint16_t us) {
    htim->Instance->CNT = 0;
    HAL_TIM_Base_Start(htim);
    while (htim->Instance->CNT < us);
    HAL_TIM_Base_Stop(htim);
}

// Hàm tạo xung tín hiệu cho chân Trigger
static void pulseGPIO(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    Delay_us(10);  // Tạo xung 10 micro giây
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

// Bắt đầu đo khoảng cách
void HCSR04_Start(void) {
    if (hc04_state == HCSR04_IDLE_STATE) {
        pulseGPIO(GPIOA, GPIO_PIN_8);  // Trigger trên chân PA8
        hc04_state = HCSR04_WAIT_RISING_STATE;
    }
}

// Xử lý trạng thái hoàn thành đo
int HCSR04_Handle(void) {
    if (hc04_state == HCSR04_COMPLETE_STATE) {
        hcsr04_distance = 0.017f * htim->Instance->CNT;  // Tính khoảng cách
        HCSR04_Complete_Callback(hcsr04_distance);
        hc04_state = HCSR04_IDLE_STATE;
        return 1;
    }
    return 0;
}

// Callback EXTI khi xảy ra ngắt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (hc04_state) {
        case HCSR04_WAIT_RISING_STATE:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
                htim->Instance->CNT = 0;
                hc04_state = HCSRO4_WAIT_FALLING_STATE;
                HAL_TIM_Base_Start(htim);
            } else {
                hc04_state = HCSR04_IDLE_STATE;
            }
            break;

        case HCSRO4_WAIT_FALLING_STATE:
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
                HAL_TIM_Base_Stop(htim);
                hc04_state = HCSR04_COMPLETE_STATE;
            } else {
                hc04_state = HCSR04_IDLE_STATE;
            }
            break;

        default:
            break;
    }
}

// Callback khi đo xong khoảng cách
__weak void HCSR04_Complete_Callback(float measured_hcsr04_distance) {
    // Hàm này được định nghĩa lại trong user file
}
