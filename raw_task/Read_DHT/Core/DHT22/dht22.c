#include "dht22.h"
//--------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
micros *= (SystemCoreClock / 1000000) / 9;
/* Wait till done */
while (micros--) ;
}

//--------------------------------------------------
void dht22_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};	
  HAL_GPIO_DeInit(data_port, data_pin);
  GPIO_InitStruct.Pin = data_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(data_port, &GPIO_InitStruct);
	///////////////////////////////////////////////////////
  HAL_Delay(1000);
  HAL_GPIO_WritePin(data_port,data_pin,GPIO_PIN_SET);

}

uint8_t DHT22_GetTemp_Humidity(float *Temp, float *Humidity)
{
	uint8_t dataArray[6], myChecksum;
	uint16_t Temp16, Humid16;
	uint8_t i, j = 0;
	HAL_GPIO_WritePin(data_port,data_pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(data_port,data_pin,GPIO_PIN_SET);
	DelayMicro(2000);
	HAL_GPIO_WritePin(data_port,data_pin,GPIO_PIN_RESET);
	DelayMicro(1200);
	HAL_GPIO_WritePin(data_port,data_pin,GPIO_PIN_SET);	
	DelayMicro(39);//20-40 	
	if(HAL_GPIO_ReadPin(data_port,data_pin)==GPIO_PIN_SET)
	{
		return 0;
	}
	DelayMicro(80);
	if(HAL_GPIO_ReadPin(data_port,data_pin)==GPIO_PIN_RESET)
	{
		return 0;
	}
	DelayMicro(80);

	for (j=0; j<5; j++)
	{
		dataArray[j]=0;
		for(i=0; i<8; i++)
		{
			while((HAL_GPIO_ReadPin(data_port,data_pin))==GPIO_PIN_RESET);	
			DelayMicro(30);
				if(HAL_GPIO_ReadPin(data_port,data_pin))
				dataArray[j] |= (1<<(7-i));
			while((HAL_GPIO_ReadPin(data_port,data_pin))==GPIO_PIN_SET);				
		}
	}
	myChecksum = 0;
	for(uint8_t k=0; k<4; k++) 
	{
		myChecksum += dataArray[k];
	}
		Temp16 = (dataArray[2] <<8) | dataArray[3];
		Humid16 = (dataArray[0] <<8) | dataArray[1];
		
		*Temp = Temp16/10.0f;
		*Humidity = Humid16/10.0f;
		return 1;
}

//----------------------------------------------------------
