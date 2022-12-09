################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/debug.c \
../Core/lib/gps.c \
../Core/lib/maths.c \
../Core/lib/sensor.c 

OBJS += \
./Core/lib/debug.o \
./Core/lib/gps.o \
./Core/lib/maths.o \
./Core/lib/sensor.o 

C_DEPS += \
./Core/lib/debug.d \
./Core/lib/gps.d \
./Core/lib/maths.d \
./Core/lib/sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/%.o Core/lib/%.su: ../Core/lib/%.c Core/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/sudo/Desktop/mario_uav/Core/flymode/quadrotor" -I../Core/Inc -I"C:/Users/sudo/Documents/stm_workspace/m/Core/lib" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-lib

clean-Core-2f-lib:
	-$(RM) ./Core/lib/debug.d ./Core/lib/debug.o ./Core/lib/debug.su ./Core/lib/gps.d ./Core/lib/gps.o ./Core/lib/gps.su ./Core/lib/maths.d ./Core/lib/maths.o ./Core/lib/maths.su ./Core/lib/sensor.d ./Core/lib/sensor.o ./Core/lib/sensor.su

.PHONY: clean-Core-2f-lib

