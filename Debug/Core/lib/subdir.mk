################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/bmp280.c \
../Core/lib/debug.c \
../Core/lib/gps.c \
../Core/lib/maths.c \
../Core/lib/mpu6500.c \
../Core/lib/pid.c \
../Core/lib/pwmwrite.c \
../Core/lib/qmc5883.c 

OBJS += \
./Core/lib/bmp280.o \
./Core/lib/debug.o \
./Core/lib/gps.o \
./Core/lib/maths.o \
./Core/lib/mpu6500.o \
./Core/lib/pid.o \
./Core/lib/pwmwrite.o \
./Core/lib/qmc5883.o 

C_DEPS += \
./Core/lib/bmp280.d \
./Core/lib/debug.d \
./Core/lib/gps.d \
./Core/lib/maths.d \
./Core/lib/mpu6500.d \
./Core/lib/pid.d \
./Core/lib/pwmwrite.d \
./Core/lib/qmc5883.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/%.o Core/lib/%.su: ../Core/lib/%.c Core/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/sudo/Documents/GitHub/orionflight/Core/flymode/quadrotor" -I"C:/Users/sudo/Documents/GitHub/orionflight/Core/lib" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-lib

clean-Core-2f-lib:
	-$(RM) ./Core/lib/bmp280.d ./Core/lib/bmp280.o ./Core/lib/bmp280.su ./Core/lib/debug.d ./Core/lib/debug.o ./Core/lib/debug.su ./Core/lib/gps.d ./Core/lib/gps.o ./Core/lib/gps.su ./Core/lib/maths.d ./Core/lib/maths.o ./Core/lib/maths.su ./Core/lib/mpu6500.d ./Core/lib/mpu6500.o ./Core/lib/mpu6500.su ./Core/lib/pid.d ./Core/lib/pid.o ./Core/lib/pid.su ./Core/lib/pwmwrite.d ./Core/lib/pwmwrite.o ./Core/lib/pwmwrite.su ./Core/lib/qmc5883.d ./Core/lib/qmc5883.o ./Core/lib/qmc5883.su

.PHONY: clean-Core-2f-lib

