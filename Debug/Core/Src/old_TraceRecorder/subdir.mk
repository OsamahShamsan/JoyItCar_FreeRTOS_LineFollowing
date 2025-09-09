################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/old_TraceRecorder/trcAssert.c \
../Core/Src/old_TraceRecorder/trcCounter.c \
../Core/Src/old_TraceRecorder/trcDependency.c \
../Core/Src/old_TraceRecorder/trcDiagnostics.c \
../Core/Src/old_TraceRecorder/trcEntryTable.c \
../Core/Src/old_TraceRecorder/trcError.c \
../Core/Src/old_TraceRecorder/trcEvent.c \
../Core/Src/old_TraceRecorder/trcEventBuffer.c \
../Core/Src/old_TraceRecorder/trcExtension.c \
../Core/Src/old_TraceRecorder/trcHardwarePort.c \
../Core/Src/old_TraceRecorder/trcHeap.c \
../Core/Src/old_TraceRecorder/trcISR.c \
../Core/Src/old_TraceRecorder/trcInternalEventBuffer.c \
../Core/Src/old_TraceRecorder/trcInterval.c \
../Core/Src/old_TraceRecorder/trcKernelPort.c \
../Core/Src/old_TraceRecorder/trcMultiCoreEventBuffer.c \
../Core/Src/old_TraceRecorder/trcObject.c \
../Core/Src/old_TraceRecorder/trcPrint.c \
../Core/Src/old_TraceRecorder/trcRunnable.c \
../Core/Src/old_TraceRecorder/trcSnapshotRecorder.c \
../Core/Src/old_TraceRecorder/trcStackMonitor.c \
../Core/Src/old_TraceRecorder/trcStateMachine.c \
../Core/Src/old_TraceRecorder/trcStaticBuffer.c \
../Core/Src/old_TraceRecorder/trcStreamingRecorder.c \
../Core/Src/old_TraceRecorder/trcString.c \
../Core/Src/old_TraceRecorder/trcTask.c \
../Core/Src/old_TraceRecorder/trcTimestamp.c 

OBJS += \
./Core/Src/old_TraceRecorder/trcAssert.o \
./Core/Src/old_TraceRecorder/trcCounter.o \
./Core/Src/old_TraceRecorder/trcDependency.o \
./Core/Src/old_TraceRecorder/trcDiagnostics.o \
./Core/Src/old_TraceRecorder/trcEntryTable.o \
./Core/Src/old_TraceRecorder/trcError.o \
./Core/Src/old_TraceRecorder/trcEvent.o \
./Core/Src/old_TraceRecorder/trcEventBuffer.o \
./Core/Src/old_TraceRecorder/trcExtension.o \
./Core/Src/old_TraceRecorder/trcHardwarePort.o \
./Core/Src/old_TraceRecorder/trcHeap.o \
./Core/Src/old_TraceRecorder/trcISR.o \
./Core/Src/old_TraceRecorder/trcInternalEventBuffer.o \
./Core/Src/old_TraceRecorder/trcInterval.o \
./Core/Src/old_TraceRecorder/trcKernelPort.o \
./Core/Src/old_TraceRecorder/trcMultiCoreEventBuffer.o \
./Core/Src/old_TraceRecorder/trcObject.o \
./Core/Src/old_TraceRecorder/trcPrint.o \
./Core/Src/old_TraceRecorder/trcRunnable.o \
./Core/Src/old_TraceRecorder/trcSnapshotRecorder.o \
./Core/Src/old_TraceRecorder/trcStackMonitor.o \
./Core/Src/old_TraceRecorder/trcStateMachine.o \
./Core/Src/old_TraceRecorder/trcStaticBuffer.o \
./Core/Src/old_TraceRecorder/trcStreamingRecorder.o \
./Core/Src/old_TraceRecorder/trcString.o \
./Core/Src/old_TraceRecorder/trcTask.o \
./Core/Src/old_TraceRecorder/trcTimestamp.o 

C_DEPS += \
./Core/Src/old_TraceRecorder/trcAssert.d \
./Core/Src/old_TraceRecorder/trcCounter.d \
./Core/Src/old_TraceRecorder/trcDependency.d \
./Core/Src/old_TraceRecorder/trcDiagnostics.d \
./Core/Src/old_TraceRecorder/trcEntryTable.d \
./Core/Src/old_TraceRecorder/trcError.d \
./Core/Src/old_TraceRecorder/trcEvent.d \
./Core/Src/old_TraceRecorder/trcEventBuffer.d \
./Core/Src/old_TraceRecorder/trcExtension.d \
./Core/Src/old_TraceRecorder/trcHardwarePort.d \
./Core/Src/old_TraceRecorder/trcHeap.d \
./Core/Src/old_TraceRecorder/trcISR.d \
./Core/Src/old_TraceRecorder/trcInternalEventBuffer.d \
./Core/Src/old_TraceRecorder/trcInterval.d \
./Core/Src/old_TraceRecorder/trcKernelPort.d \
./Core/Src/old_TraceRecorder/trcMultiCoreEventBuffer.d \
./Core/Src/old_TraceRecorder/trcObject.d \
./Core/Src/old_TraceRecorder/trcPrint.d \
./Core/Src/old_TraceRecorder/trcRunnable.d \
./Core/Src/old_TraceRecorder/trcSnapshotRecorder.d \
./Core/Src/old_TraceRecorder/trcStackMonitor.d \
./Core/Src/old_TraceRecorder/trcStateMachine.d \
./Core/Src/old_TraceRecorder/trcStaticBuffer.d \
./Core/Src/old_TraceRecorder/trcStreamingRecorder.d \
./Core/Src/old_TraceRecorder/trcString.d \
./Core/Src/old_TraceRecorder/trcTask.d \
./Core/Src/old_TraceRecorder/trcTimestamp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/old_TraceRecorder/%.o Core/Src/old_TraceRecorder/%.su Core/Src/old_TraceRecorder/%.cyclo: ../Core/Src/old_TraceRecorder/%.c Core/Src/old_TraceRecorder/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/include" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/config" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM/config" -I"C:/Users/engsh/Desktop/OS_SEES_SS25/Lab5_JoyItCar_Shamsan/Core/Src/TraceRecorder/streamports/ARM_ITM/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-old_TraceRecorder

clean-Core-2f-Src-2f-old_TraceRecorder:
	-$(RM) ./Core/Src/old_TraceRecorder/trcAssert.cyclo ./Core/Src/old_TraceRecorder/trcAssert.d ./Core/Src/old_TraceRecorder/trcAssert.o ./Core/Src/old_TraceRecorder/trcAssert.su ./Core/Src/old_TraceRecorder/trcCounter.cyclo ./Core/Src/old_TraceRecorder/trcCounter.d ./Core/Src/old_TraceRecorder/trcCounter.o ./Core/Src/old_TraceRecorder/trcCounter.su ./Core/Src/old_TraceRecorder/trcDependency.cyclo ./Core/Src/old_TraceRecorder/trcDependency.d ./Core/Src/old_TraceRecorder/trcDependency.o ./Core/Src/old_TraceRecorder/trcDependency.su ./Core/Src/old_TraceRecorder/trcDiagnostics.cyclo ./Core/Src/old_TraceRecorder/trcDiagnostics.d ./Core/Src/old_TraceRecorder/trcDiagnostics.o ./Core/Src/old_TraceRecorder/trcDiagnostics.su ./Core/Src/old_TraceRecorder/trcEntryTable.cyclo ./Core/Src/old_TraceRecorder/trcEntryTable.d ./Core/Src/old_TraceRecorder/trcEntryTable.o ./Core/Src/old_TraceRecorder/trcEntryTable.su ./Core/Src/old_TraceRecorder/trcError.cyclo ./Core/Src/old_TraceRecorder/trcError.d ./Core/Src/old_TraceRecorder/trcError.o ./Core/Src/old_TraceRecorder/trcError.su ./Core/Src/old_TraceRecorder/trcEvent.cyclo ./Core/Src/old_TraceRecorder/trcEvent.d ./Core/Src/old_TraceRecorder/trcEvent.o ./Core/Src/old_TraceRecorder/trcEvent.su ./Core/Src/old_TraceRecorder/trcEventBuffer.cyclo ./Core/Src/old_TraceRecorder/trcEventBuffer.d ./Core/Src/old_TraceRecorder/trcEventBuffer.o ./Core/Src/old_TraceRecorder/trcEventBuffer.su ./Core/Src/old_TraceRecorder/trcExtension.cyclo ./Core/Src/old_TraceRecorder/trcExtension.d ./Core/Src/old_TraceRecorder/trcExtension.o ./Core/Src/old_TraceRecorder/trcExtension.su ./Core/Src/old_TraceRecorder/trcHardwarePort.cyclo ./Core/Src/old_TraceRecorder/trcHardwarePort.d ./Core/Src/old_TraceRecorder/trcHardwarePort.o ./Core/Src/old_TraceRecorder/trcHardwarePort.su ./Core/Src/old_TraceRecorder/trcHeap.cyclo ./Core/Src/old_TraceRecorder/trcHeap.d ./Core/Src/old_TraceRecorder/trcHeap.o ./Core/Src/old_TraceRecorder/trcHeap.su ./Core/Src/old_TraceRecorder/trcISR.cyclo ./Core/Src/old_TraceRecorder/trcISR.d ./Core/Src/old_TraceRecorder/trcISR.o ./Core/Src/old_TraceRecorder/trcISR.su ./Core/Src/old_TraceRecorder/trcInternalEventBuffer.cyclo ./Core/Src/old_TraceRecorder/trcInternalEventBuffer.d ./Core/Src/old_TraceRecorder/trcInternalEventBuffer.o ./Core/Src/old_TraceRecorder/trcInternalEventBuffer.su ./Core/Src/old_TraceRecorder/trcInterval.cyclo ./Core/Src/old_TraceRecorder/trcInterval.d ./Core/Src/old_TraceRecorder/trcInterval.o ./Core/Src/old_TraceRecorder/trcInterval.su ./Core/Src/old_TraceRecorder/trcKernelPort.cyclo ./Core/Src/old_TraceRecorder/trcKernelPort.d ./Core/Src/old_TraceRecorder/trcKernelPort.o ./Core/Src/old_TraceRecorder/trcKernelPort.su ./Core/Src/old_TraceRecorder/trcMultiCoreEventBuffer.cyclo ./Core/Src/old_TraceRecorder/trcMultiCoreEventBuffer.d ./Core/Src/old_TraceRecorder/trcMultiCoreEventBuffer.o ./Core/Src/old_TraceRecorder/trcMultiCoreEventBuffer.su ./Core/Src/old_TraceRecorder/trcObject.cyclo ./Core/Src/old_TraceRecorder/trcObject.d ./Core/Src/old_TraceRecorder/trcObject.o ./Core/Src/old_TraceRecorder/trcObject.su ./Core/Src/old_TraceRecorder/trcPrint.cyclo ./Core/Src/old_TraceRecorder/trcPrint.d ./Core/Src/old_TraceRecorder/trcPrint.o ./Core/Src/old_TraceRecorder/trcPrint.su ./Core/Src/old_TraceRecorder/trcRunnable.cyclo ./Core/Src/old_TraceRecorder/trcRunnable.d ./Core/Src/old_TraceRecorder/trcRunnable.o ./Core/Src/old_TraceRecorder/trcRunnable.su ./Core/Src/old_TraceRecorder/trcSnapshotRecorder.cyclo ./Core/Src/old_TraceRecorder/trcSnapshotRecorder.d ./Core/Src/old_TraceRecorder/trcSnapshotRecorder.o ./Core/Src/old_TraceRecorder/trcSnapshotRecorder.su ./Core/Src/old_TraceRecorder/trcStackMonitor.cyclo ./Core/Src/old_TraceRecorder/trcStackMonitor.d ./Core/Src/old_TraceRecorder/trcStackMonitor.o ./Core/Src/old_TraceRecorder/trcStackMonitor.su ./Core/Src/old_TraceRecorder/trcStateMachine.cyclo ./Core/Src/old_TraceRecorder/trcStateMachine.d ./Core/Src/old_TraceRecorder/trcStateMachine.o ./Core/Src/old_TraceRecorder/trcStateMachine.su ./Core/Src/old_TraceRecorder/trcStaticBuffer.cyclo ./Core/Src/old_TraceRecorder/trcStaticBuffer.d ./Core/Src/old_TraceRecorder/trcStaticBuffer.o ./Core/Src/old_TraceRecorder/trcStaticBuffer.su ./Core/Src/old_TraceRecorder/trcStreamingRecorder.cyclo ./Core/Src/old_TraceRecorder/trcStreamingRecorder.d ./Core/Src/old_TraceRecorder/trcStreamingRecorder.o ./Core/Src/old_TraceRecorder/trcStreamingRecorder.su ./Core/Src/old_TraceRecorder/trcString.cyclo ./Core/Src/old_TraceRecorder/trcString.d ./Core/Src/old_TraceRecorder/trcString.o ./Core/Src/old_TraceRecorder/trcString.su ./Core/Src/old_TraceRecorder/trcTask.cyclo ./Core/Src/old_TraceRecorder/trcTask.d ./Core/Src/old_TraceRecorder/trcTask.o ./Core/Src/old_TraceRecorder/trcTask.su ./Core/Src/old_TraceRecorder/trcTimestamp.cyclo ./Core/Src/old_TraceRecorder/trcTimestamp.d ./Core/Src/old_TraceRecorder/trcTimestamp.o ./Core/Src/old_TraceRecorder/trcTimestamp.su

.PHONY: clean-Core-2f-Src-2f-old_TraceRecorder

