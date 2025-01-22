#ifndef UTIL_H_
#define UTIL_H_

#include "stm32f1xx_hal.h" // Thay đổi theo dòng STM32 của bạn

/*
*---------------------------------------
*   GPIO Direct Register Access Macros
*---------------------------------------
*/

#define GPIO_SET_PIN(port, pin) 	((port)->BSRR = (pin))
#define GPIO_CLEAR_PIN(port, pin) 	((port)->BSRR = (pin << 16u))
#define GPIO_TOGGLE_PIN(port, pin) 	((port)->ODR  ^= (pin))
#define GPIO_READ_PIN(port, pin) 	((port)->IDR & (pin))

/*
*---------------------------------------
*   Timer 1 Delay Macros
*---------------------------------------
*/

// Khai báo handle Timer 1 (phải được định nghĩa trong main.c hoặc file liên quan)
extern TIM_HandleTypeDef htim3;

#define DELAY_US(us)  \
    do { \
        uint32_t start_tick = __HAL_TIM_GET_COUNTER(&htim3);  /* Đọc giá trị bộ đếm hiện tại */ \
        uint32_t ticks = us;  /* Số ticks cần thiết cho độ trễ microseconds */ \
        while ((__HAL_TIM_GET_COUNTER(&htim3) - start_tick) < ticks); \
    } while(0)

#define DELAY_MS(ms)  \
    do { \
        for (uint32_t i = 0; i < ms; ++i) { \
            DELAY_US(1000);  /* Đợi 1ms bằng cách gọi độ trễ microseconds */ \
        } \
    } while (0)


#endif /* UTIL_H_ */
