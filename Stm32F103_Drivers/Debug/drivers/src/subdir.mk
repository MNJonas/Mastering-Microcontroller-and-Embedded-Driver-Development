################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f103xx_gpio_driver.c 

OBJS += \
./drivers/src/stm32f103xx_gpio_driver.o 

C_DEPS += \
./drivers/src/stm32f103xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/jmwam/Documents/Projects/Mastering-Microcontroller-and-Embedded-Driver-Development/Stm32F103_Drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f103xx_gpio_driver.cyclo ./drivers/src/stm32f103xx_gpio_driver.d ./drivers/src/stm32f103xx_gpio_driver.o ./drivers/src/stm32f103xx_gpio_driver.su

.PHONY: clean-drivers-2f-src

