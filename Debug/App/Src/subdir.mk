################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/app_tasks.c 

OBJS += \
./App/Src/app_tasks.o 

C_DEPS += \
./App/Src/app_tasks.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/dalya/STM32CubeIDE/FreeRTOS_ws/Fault-Tolerant-RTOS/ThirdParty/FreeRTOS/include" -I"C:/Users/dalya/STM32CubeIDE/FreeRTOS_ws/Fault-Tolerant-RTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/dalya/STM32CubeIDE/FreeRTOS_ws/Fault-Tolerant-RTOS/ThirdParty/FreeRTOS" -I"C:/Users/dalya/STM32CubeIDE/FreeRTOS_ws/Fault-Tolerant-RTOS/App/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/app_tasks.cyclo ./App/Src/app_tasks.d ./App/Src/app_tasks.o ./App/Src/app_tasks.su

.PHONY: clean-App-2f-Src

