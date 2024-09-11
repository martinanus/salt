################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/commands_logic.c \
../Core/Src/communications.c \
../Core/Src/critical_signals.c \
../Core/Src/freertos.c \
../Core/Src/front_panel.c \
../Core/Src/gps.c \
../Core/Src/led_driver.c \
../Core/Src/main.c \
../Core/Src/sd.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_inputs.c \
../Core/Src/system_mode_logic.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/testing.c 

OBJS += \
./Core/Src/commands_logic.o \
./Core/Src/communications.o \
./Core/Src/critical_signals.o \
./Core/Src/freertos.o \
./Core/Src/front_panel.o \
./Core/Src/gps.o \
./Core/Src/led_driver.o \
./Core/Src/main.o \
./Core/Src/sd.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_inputs.o \
./Core/Src/system_mode_logic.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/testing.o 

C_DEPS += \
./Core/Src/commands_logic.d \
./Core/Src/communications.d \
./Core/Src/critical_signals.d \
./Core/Src/freertos.d \
./Core/Src/front_panel.d \
./Core/Src/gps.d \
./Core/Src/led_driver.d \
./Core/Src/main.d \
./Core/Src/sd.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_inputs.d \
./Core/Src/system_mode_logic.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/testing.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/commands_logic.cyclo ./Core/Src/commands_logic.d ./Core/Src/commands_logic.o ./Core/Src/commands_logic.su ./Core/Src/communications.cyclo ./Core/Src/communications.d ./Core/Src/communications.o ./Core/Src/communications.su ./Core/Src/critical_signals.cyclo ./Core/Src/critical_signals.d ./Core/Src/critical_signals.o ./Core/Src/critical_signals.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/front_panel.cyclo ./Core/Src/front_panel.d ./Core/Src/front_panel.o ./Core/Src/front_panel.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/led_driver.cyclo ./Core/Src/led_driver.d ./Core/Src/led_driver.o ./Core/Src/led_driver.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/sd.cyclo ./Core/Src/sd.d ./Core/Src/sd.o ./Core/Src/sd.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_inputs.cyclo ./Core/Src/system_inputs.d ./Core/Src/system_inputs.o ./Core/Src/system_inputs.su ./Core/Src/system_mode_logic.cyclo ./Core/Src/system_mode_logic.d ./Core/Src/system_mode_logic.o ./Core/Src/system_mode_logic.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/testing.cyclo ./Core/Src/testing.d ./Core/Src/testing.o ./Core/Src/testing.su

.PHONY: clean-Core-2f-Src

