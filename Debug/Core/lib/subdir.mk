################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/ahrs.c \
../Core/lib/bmp280.c \
../Core/lib/gps.c \
../Core/lib/hc_sr04.c \
../Core/lib/ibus.c \
../Core/lib/imu.c \
../Core/lib/log.c \
../Core/lib/maths.c \
../Core/lib/mavlink_handler.c \
../Core/lib/mpu6500.c \
../Core/lib/ms5611.c \
../Core/lib/nrf24l01p.c \
../Core/lib/pid.c \
../Core/lib/ppmreceiver.c \
../Core/lib/pwmwrite.c \
../Core/lib/qmc5883.c 

OBJS += \
./Core/lib/ahrs.o \
./Core/lib/bmp280.o \
./Core/lib/gps.o \
./Core/lib/hc_sr04.o \
./Core/lib/ibus.o \
./Core/lib/imu.o \
./Core/lib/log.o \
./Core/lib/maths.o \
./Core/lib/mavlink_handler.o \
./Core/lib/mpu6500.o \
./Core/lib/ms5611.o \
./Core/lib/nrf24l01p.o \
./Core/lib/pid.o \
./Core/lib/ppmreceiver.o \
./Core/lib/pwmwrite.o \
./Core/lib/qmc5883.o 

C_DEPS += \
./Core/lib/ahrs.d \
./Core/lib/bmp280.d \
./Core/lib/gps.d \
./Core/lib/hc_sr04.d \
./Core/lib/ibus.d \
./Core/lib/imu.d \
./Core/lib/log.d \
./Core/lib/maths.d \
./Core/lib/mavlink_handler.d \
./Core/lib/mpu6500.d \
./Core/lib/ms5611.d \
./Core/lib/nrf24l01p.d \
./Core/lib/pid.d \
./Core/lib/ppmreceiver.d \
./Core/lib/pwmwrite.d \
./Core/lib/qmc5883.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/%.o Core/lib/%.su Core/lib/%.cyclo: ../Core/lib/%.c Core/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/tuan/Documents/GitHub/orionflight/Core/lib" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-lib

clean-Core-2f-lib:
	-$(RM) ./Core/lib/ahrs.cyclo ./Core/lib/ahrs.d ./Core/lib/ahrs.o ./Core/lib/ahrs.su ./Core/lib/bmp280.cyclo ./Core/lib/bmp280.d ./Core/lib/bmp280.o ./Core/lib/bmp280.su ./Core/lib/gps.cyclo ./Core/lib/gps.d ./Core/lib/gps.o ./Core/lib/gps.su ./Core/lib/hc_sr04.cyclo ./Core/lib/hc_sr04.d ./Core/lib/hc_sr04.o ./Core/lib/hc_sr04.su ./Core/lib/ibus.cyclo ./Core/lib/ibus.d ./Core/lib/ibus.o ./Core/lib/ibus.su ./Core/lib/imu.cyclo ./Core/lib/imu.d ./Core/lib/imu.o ./Core/lib/imu.su ./Core/lib/log.cyclo ./Core/lib/log.d ./Core/lib/log.o ./Core/lib/log.su ./Core/lib/maths.cyclo ./Core/lib/maths.d ./Core/lib/maths.o ./Core/lib/maths.su ./Core/lib/mavlink_handler.cyclo ./Core/lib/mavlink_handler.d ./Core/lib/mavlink_handler.o ./Core/lib/mavlink_handler.su ./Core/lib/mpu6500.cyclo ./Core/lib/mpu6500.d ./Core/lib/mpu6500.o ./Core/lib/mpu6500.su ./Core/lib/ms5611.cyclo ./Core/lib/ms5611.d ./Core/lib/ms5611.o ./Core/lib/ms5611.su ./Core/lib/nrf24l01p.cyclo ./Core/lib/nrf24l01p.d ./Core/lib/nrf24l01p.o ./Core/lib/nrf24l01p.su ./Core/lib/pid.cyclo ./Core/lib/pid.d ./Core/lib/pid.o ./Core/lib/pid.su ./Core/lib/ppmreceiver.cyclo ./Core/lib/ppmreceiver.d ./Core/lib/ppmreceiver.o ./Core/lib/ppmreceiver.su ./Core/lib/pwmwrite.cyclo ./Core/lib/pwmwrite.d ./Core/lib/pwmwrite.o ./Core/lib/pwmwrite.su ./Core/lib/qmc5883.cyclo ./Core/lib/qmc5883.d ./Core/lib/qmc5883.o ./Core/lib/qmc5883.su

.PHONY: clean-Core-2f-lib

