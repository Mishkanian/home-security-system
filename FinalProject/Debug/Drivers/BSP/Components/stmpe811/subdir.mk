################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/stmpe811/stmpe811.c 

OBJS += \
./Drivers/BSP/Components/stmpe811/stmpe811.o 

C_DEPS += \
./Drivers/BSP/Components/stmpe811/stmpe811.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/stmpe811/%.o Drivers/BSP/Components/stmpe811/%.su Drivers/BSP/Components/stmpe811/%.cyclo: ../Drivers/BSP/Components/stmpe811/%.c Drivers/BSP/Components/stmpe811/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I/Users/mjm/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc -I/Users/mjm/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include -I/Users/mjm/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/BSP/B-L475E-IOT01 -I/Users/mjm/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -I/Users/mjm/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Projects/B-L475E-IOT01A/Applications/WiFi/Common/Inc -I/Users/mjm/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Projects/B-L475E-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-stmpe811

clean-Drivers-2f-BSP-2f-Components-2f-stmpe811:
	-$(RM) ./Drivers/BSP/Components/stmpe811/stmpe811.cyclo ./Drivers/BSP/Components/stmpe811/stmpe811.d ./Drivers/BSP/Components/stmpe811/stmpe811.o ./Drivers/BSP/Components/stmpe811/stmpe811.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-stmpe811

