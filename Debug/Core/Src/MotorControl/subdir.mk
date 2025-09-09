################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MotorControl/motor.c 

OBJS += \
./Core/Src/MotorControl/motor.o 

C_DEPS += \
./Core/Src/MotorControl/motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MotorControl/%.o Core/Src/MotorControl/%.su Core/Src/MotorControl/%.cyclo: ../Core/Src/MotorControl/%.c Core/Src/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/MotorControl" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/config" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/include" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM/include" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM/config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MotorControl

clean-Core-2f-Src-2f-MotorControl:
	-$(RM) ./Core/Src/MotorControl/motor.cyclo ./Core/Src/MotorControl/motor.d ./Core/Src/MotorControl/motor.o ./Core/Src/MotorControl/motor.su

.PHONY: clean-Core-2f-Src-2f-MotorControl

