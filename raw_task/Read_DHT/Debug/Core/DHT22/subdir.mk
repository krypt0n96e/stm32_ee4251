################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/DHT22/dht22.c 

OBJS += \
./Core/DHT22/dht22.o 

C_DEPS += \
./Core/DHT22/dht22.d 


# Each subdirectory must supply rules for building sources it contributes
Core/DHT22/%.o Core/DHT22/%.su Core/DHT22/%.cyclo: ../Core/DHT22/%.c Core/DHT22/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/STM32IDE/Read_DHT/Core/DHT22" -I"C:/STM32IDE/Read_DHT/Core/LIBDHT11" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-DHT22

clean-Core-2f-DHT22:
	-$(RM) ./Core/DHT22/dht22.cyclo ./Core/DHT22/dht22.d ./Core/DHT22/dht22.o ./Core/DHT22/dht22.su

.PHONY: clean-Core-2f-DHT22

