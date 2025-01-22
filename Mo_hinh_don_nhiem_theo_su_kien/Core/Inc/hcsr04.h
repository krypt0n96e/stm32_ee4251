#ifndef HCSR04_H
#define HCSR04_H

#include "stm32f1xx_hal.h"

// Enum trạng thái của cảm biến HCSR04
typedef enum {
    HCSR04_IDLE_STATE,
    HCSR04_WAIT_RISING_STATE,
    HCSRO4_WAIT_FALLING_STATE,
    HCSR04_COMPLETE_STATE
} HCSR04_State;

// Hàm khởi tạo
void HCSR04_Init(TIM_HandleTypeDef *timer);

// Bắt đầu đo khoảng cách
void HCSR04_Start(void);

// Xử lý trạng thái hoàn thành đo
void HCSR04_Handle(void);

// Callback khi hoàn thành đo khoảng cách
void HCSR04_Complete_Callback(float measured_distance);

// Biến toàn cục dùng trong main.c
extern HCSR04_State hc04_state;
extern TIM_HandleTypeDef *htim;
extern float hcsr04_distance;

#endif // HCSR04_H
