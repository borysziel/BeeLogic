################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SS_MS5607.c \
../Src/SS_S25FL.c \
../Src/SS_SCD30.c \
../Src/SS_it.c \
../Src/SS_support.c \
../Src/adc.c \
../Src/dma.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/rtc.c \
../Src/scd_git_version.c \
../Src/sensirion_common.c \
../Src/sensirion_hw_i2c_implementation.c \
../Src/spi.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/SS_MS5607.o \
./Src/SS_S25FL.o \
./Src/SS_SCD30.o \
./Src/SS_it.o \
./Src/SS_support.o \
./Src/adc.o \
./Src/dma.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/rtc.o \
./Src/scd_git_version.o \
./Src/sensirion_common.o \
./Src/sensirion_hw_i2c_implementation.o \
./Src/spi.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/SS_MS5607.d \
./Src/SS_S25FL.d \
./Src/SS_SCD30.d \
./Src/SS_it.d \
./Src/SS_support.d \
./Src/adc.d \
./Src/dma.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/rtc.d \
./Src/scd_git_version.d \
./Src/sensirion_common.d \
./Src/sensirion_hw_i2c_implementation.d \
./Src/spi.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"D:/Elektronika_programowanie/SPACE SYSTEMS/BeeLogic3/Inc" -I"D:/Elektronika_programowanie/SPACE SYSTEMS/BeeLogic3/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/Elektronika_programowanie/SPACE SYSTEMS/BeeLogic3/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/Elektronika_programowanie/SPACE SYSTEMS/BeeLogic3/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/Elektronika_programowanie/SPACE SYSTEMS/BeeLogic3/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


