################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/myJoyCar/SensorsControl/sensors.c 

OBJS += \
./Core/Src/myJoyCar/SensorsControl/sensors.o 

C_DEPS += \
./Core/Src/myJoyCar/SensorsControl/sensors.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/myJoyCar/SensorsControl/%.o Core/Src/myJoyCar/SensorsControl/%.su Core/Src/myJoyCar/SensorsControl/%.cyclo: ../Core/Src/myJoyCar/SensorsControl/%.c Core/Src/myJoyCar/SensorsControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/config" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/include" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM/include" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM/config" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/myJoyCar" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/myJoyCar/MotorControl" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/myJoyCar/SensorsControl" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-myJoyCar-2f-SensorsControl

clean-Core-2f-Src-2f-myJoyCar-2f-SensorsControl:
	-$(RM) ./Core/Src/myJoyCar/SensorsControl/sensors.cyclo ./Core/Src/myJoyCar/SensorsControl/sensors.d ./Core/Src/myJoyCar/SensorsControl/sensors.o ./Core/Src/myJoyCar/SensorsControl/sensors.su

.PHONY: clean-Core-2f-Src-2f-myJoyCar-2f-SensorsControl

