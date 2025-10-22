################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f446xx_gpio_driver.c 

OBJS += \
./drivers/Src/stm32f446xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32f446xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"D:/STM/chip_headers/Drivers/CMSIS/Include" -I"D:/STM/chip_headers/Drivers/CMSIS/Core/Include" -I"D:/STM/chip_headers/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/STM/chip_headers/Drivers/CMSIS/Device/ST/STM32F4xx/Source" -I"D:/STM/GuitarTuner/Middlewares/ST/ARM/DSP/Inc" -I"D:/STM/GuitarTuner/Middlewares/ST/ARM/DSP/Lib" -I"D:/STM/PrecisionGuitarTuner/Inc" -I"D:/STM/PrecisionGuitarTuner/Src" -I"D:/STM/PrecisionGuitarTuner/DSP/Inc" -O0 -ffunction-sections -fdata-sections -Wall -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f446xx_gpio_driver.cyclo ./drivers/Src/stm32f446xx_gpio_driver.d ./drivers/Src/stm32f446xx_gpio_driver.o ./drivers/Src/stm32f446xx_gpio_driver.su

.PHONY: clean-drivers-2f-Src

