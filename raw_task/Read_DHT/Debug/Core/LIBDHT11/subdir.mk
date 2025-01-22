################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/LIBDHT11/DHT.c 

OBJS += \
./Core/LIBDHT11/DHT.o 

C_DEPS += \
./Core/LIBDHT11/DHT.d 


# Each subdirectory must supply rules for building sources it contributes
Core/LIBDHT11/%.o Core/LIBDHT11/%.su Core/LIBDHT11/%.cyclo: ../Core/LIBDHT11/%.c Core/LIBDHT11/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/STM32IDE/Read_DHT/Core/DHT22" -I"C:/STM32IDE/Read_DHT/Core/LIBDHT11" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-LIBDHT11

clean-Core-2f-LIBDHT11:
	-$(RM) ./Core/LIBDHT11/DHT.cyclo ./Core/LIBDHT11/DHT.d ./Core/LIBDHT11/DHT.o ./Core/LIBDHT11/DHT.su

.PHONY: clean-Core-2f-LIBDHT11

