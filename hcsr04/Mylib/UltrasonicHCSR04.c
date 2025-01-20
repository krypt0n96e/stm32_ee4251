#include "UltrasonicHCSR04.h"


HCSR04_State hc04_state = HCSR04_IDLE_STATE;

void Delay_us(uint16_t us){
	htim1.Instance -> CNT = 0;
	//TIM2 -> CNT = 0; (Cach 2)
	HAL_TIM_Base_Start(&htim1);
	while (htim1.Instance -> CNT < us);
	HAL_TIM_Base_Stop(&htim1);
	}

void Delay_ms(uint16_t ms){
	for (int i = 0;i<ms;i++){
		Delay_us(1000);
	}
}

void pulseGPIO(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
	Delay_us(20);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
}

void HCSR04_Start(){
	if (hc04_state == HCSR04_IDLE_STATE){
	pulseGPIO(GPIOA, GPIO_PIN_8);
	hc04_state = HCSR04_WAIT_RISING_STATE;
	}
}
