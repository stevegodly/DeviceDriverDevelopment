################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f407xx_gpio.c \
../drivers/src/stm32f407xx_i2c.c \
../drivers/src/stm32f407xx_rcc.c \
../drivers/src/stm32f407xx_spi.c \
../drivers/src/stm32f407xx_usart.c 

OBJS += \
./drivers/src/stm32f407xx_gpio.o \
./drivers/src/stm32f407xx_i2c.o \
./drivers/src/stm32f407xx_rcc.o \
./drivers/src/stm32f407xx_spi.o \
./drivers/src/stm32f407xx_usart.o 

C_DEPS += \
./drivers/src/stm32f407xx_gpio.d \
./drivers/src/stm32f407xx_i2c.d \
./drivers/src/stm32f407xx_rcc.d \
./drivers/src/stm32f407xx_spi.d \
./drivers/src/stm32f407xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/user.USER/Documents/Coding/c_programming/STM32/Tutorial/stm32f407xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f407xx_gpio.cyclo ./drivers/src/stm32f407xx_gpio.d ./drivers/src/stm32f407xx_gpio.o ./drivers/src/stm32f407xx_gpio.su ./drivers/src/stm32f407xx_i2c.cyclo ./drivers/src/stm32f407xx_i2c.d ./drivers/src/stm32f407xx_i2c.o ./drivers/src/stm32f407xx_i2c.su ./drivers/src/stm32f407xx_rcc.cyclo ./drivers/src/stm32f407xx_rcc.d ./drivers/src/stm32f407xx_rcc.o ./drivers/src/stm32f407xx_rcc.su ./drivers/src/stm32f407xx_spi.cyclo ./drivers/src/stm32f407xx_spi.d ./drivers/src/stm32f407xx_spi.o ./drivers/src/stm32f407xx_spi.su ./drivers/src/stm32f407xx_usart.cyclo ./drivers/src/stm32f407xx_usart.d ./drivers/src/stm32f407xx_usart.o ./drivers/src/stm32f407xx_usart.su

.PHONY: clean-drivers-2f-src

