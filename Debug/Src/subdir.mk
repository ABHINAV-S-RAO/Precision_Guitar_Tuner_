################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/lcd.c \
../Src/main.c \
../Src/stm32f446xx_gpio_driver.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/systic.c \
../Src/uart.c 

OBJS += \
./Src/adc.o \
./Src/lcd.o \
./Src/main.o \
./Src/stm32f446xx_gpio_driver.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/systic.o \
./Src/uart.o 

C_DEPS += \
./Src/adc.d \
./Src/lcd.d \
./Src/main.d \
./Src/stm32f446xx_gpio_driver.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/systic.d \
./Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"D:/STM/chip_headers/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/STM/chip_headers/Drivers/CMSIS/Device/ST/STM32F4xx/Source" -I"D:/STM/chip_headers/Drivers/CMSIS/Include" -I"D:/STM/PrecisionGuitarTuner/Inc" -I"D:/STM/PrecisionGuitarTuner/Src" -I"D:/STM/PrecisionGuitarTuner/DSP/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/adc.cyclo ./Src/adc.d ./Src/adc.o ./Src/adc.su ./Src/lcd.cyclo ./Src/lcd.d ./Src/lcd.o ./Src/lcd.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/stm32f446xx_gpio_driver.cyclo ./Src/stm32f446xx_gpio_driver.d ./Src/stm32f446xx_gpio_driver.o ./Src/stm32f446xx_gpio_driver.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/systic.cyclo ./Src/systic.d ./Src/systic.o ./Src/systic.su ./Src/uart.cyclo ./Src/uart.d ./Src/uart.o ./Src/uart.su

.PHONY: clean-Src

