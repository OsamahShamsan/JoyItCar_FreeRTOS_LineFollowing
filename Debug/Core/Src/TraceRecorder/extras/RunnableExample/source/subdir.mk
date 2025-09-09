################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/TraceRecorder/extras/RunnableExample/source/main.c 

OBJS += \
./Core/Src/TraceRecorder/extras/RunnableExample/source/main.o 

C_DEPS += \
./Core/Src/TraceRecorder/extras/RunnableExample/source/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/TraceRecorder/extras/RunnableExample/source/%.o Core/Src/TraceRecorder/extras/RunnableExample/source/%.su Core/Src/TraceRecorder/extras/RunnableExample/source/%.cyclo: ../Core/Src/TraceRecorder/extras/RunnableExample/source/%.c Core/Src/TraceRecorder/extras/RunnableExample/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/MotorControl" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-TraceRecorder-2f-extras-2f-RunnableExample-2f-source

clean-Core-2f-Src-2f-TraceRecorder-2f-extras-2f-RunnableExample-2f-source:
	-$(RM) ./Core/Src/TraceRecorder/extras/RunnableExample/source/main.cyclo ./Core/Src/TraceRecorder/extras/RunnableExample/source/main.d ./Core/Src/TraceRecorder/extras/RunnableExample/source/main.o ./Core/Src/TraceRecorder/extras/RunnableExample/source/main.su

.PHONY: clean-Core-2f-Src-2f-TraceRecorder-2f-extras-2f-RunnableExample-2f-source

