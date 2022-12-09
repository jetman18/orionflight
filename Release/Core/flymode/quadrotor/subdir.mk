################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/flymode/quadrotor/mainloop.c 

OBJS += \
./Core/flymode/quadrotor/mainloop.o 

C_DEPS += \
./Core/flymode/quadrotor/mainloop.d 


# Each subdirectory must supply rules for building sources it contributes
Core/flymode/quadrotor/%.o Core/flymode/quadrotor/%.su: ../Core/flymode/quadrotor/%.c Core/flymode/quadrotor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-flymode-2f-quadrotor

clean-Core-2f-flymode-2f-quadrotor:
	-$(RM) ./Core/flymode/quadrotor/mainloop.d ./Core/flymode/quadrotor/mainloop.o ./Core/flymode/quadrotor/mainloop.su

.PHONY: clean-Core-2f-flymode-2f-quadrotor

