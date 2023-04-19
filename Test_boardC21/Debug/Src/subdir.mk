################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/drone_controller.c \
../Src/main.c \
../Src/mpu9255.c \
../Src/nrf24.c \
../Src/pid.c \
../Src/radio_demo.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/drone_controller.o \
./Src/main.o \
./Src/mpu9255.o \
./Src/nrf24.o \
./Src/pid.o \
./Src/radio_demo.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/drone_controller.d \
./Src/main.d \
./Src/mpu9255.d \
./Src/nrf24.d \
./Src/pid.d \
./Src/radio_demo.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/drone_controller.cyclo ./Src/drone_controller.d ./Src/drone_controller.o ./Src/drone_controller.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/mpu9255.cyclo ./Src/mpu9255.d ./Src/mpu9255.o ./Src/mpu9255.su ./Src/nrf24.cyclo ./Src/nrf24.d ./Src/nrf24.o ./Src/nrf24.su ./Src/pid.cyclo ./Src/pid.d ./Src/pid.o ./Src/pid.su ./Src/radio_demo.cyclo ./Src/radio_demo.d ./Src/radio_demo.o ./Src/radio_demo.su ./Src/stm32f1xx_hal_msp.cyclo ./Src/stm32f1xx_hal_msp.d ./Src/stm32f1xx_hal_msp.o ./Src/stm32f1xx_hal_msp.su ./Src/stm32f1xx_it.cyclo ./Src/stm32f1xx_it.d ./Src/stm32f1xx_it.o ./Src/stm32f1xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f1xx.cyclo ./Src/system_stm32f1xx.d ./Src/system_stm32f1xx.o ./Src/system_stm32f1xx.su

.PHONY: clean-Src

