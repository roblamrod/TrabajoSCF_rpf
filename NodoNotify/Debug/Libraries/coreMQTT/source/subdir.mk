################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/coreMQTT/source/core_mqtt.c \
../Libraries/coreMQTT/source/core_mqtt_serializer.c \
../Libraries/coreMQTT/source/core_mqtt_state.c 

OBJS += \
./Libraries/coreMQTT/source/core_mqtt.o \
./Libraries/coreMQTT/source/core_mqtt_serializer.o \
./Libraries/coreMQTT/source/core_mqtt_state.o 

C_DEPS += \
./Libraries/coreMQTT/source/core_mqtt.d \
./Libraries/coreMQTT/source/core_mqtt_serializer.d \
./Libraries/coreMQTT/source/core_mqtt_state.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/coreMQTT/source/%.o Libraries/coreMQTT/source/%.su Libraries/coreMQTT/source/%.cyclo: ../Libraries/coreMQTT/source/%.c Libraries/coreMQTT/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"C:/Users/pablo/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_rpf/NodoNotify/L4_IOT_Sensors/Drivers/BSP/Components/lsm6dsl" -I"C:/Users/pablo/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_rpf/NodoNotify/L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01" -I"C:/Users/pablo/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_rpf/NodoNotify/L4_IOT_Sensors/Drivers/BSP/Components/hts221" -I"C:/Users/pablo/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_rpf/NodoNotify/Libraries/WIFI" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/pablo/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_rpf/NodoNotify/Libraries/coreMQTT/source" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-coreMQTT-2f-source

clean-Libraries-2f-coreMQTT-2f-source:
	-$(RM) ./Libraries/coreMQTT/source/core_mqtt.cyclo ./Libraries/coreMQTT/source/core_mqtt.d ./Libraries/coreMQTT/source/core_mqtt.o ./Libraries/coreMQTT/source/core_mqtt.su ./Libraries/coreMQTT/source/core_mqtt_serializer.cyclo ./Libraries/coreMQTT/source/core_mqtt_serializer.d ./Libraries/coreMQTT/source/core_mqtt_serializer.o ./Libraries/coreMQTT/source/core_mqtt_serializer.su ./Libraries/coreMQTT/source/core_mqtt_state.cyclo ./Libraries/coreMQTT/source/core_mqtt_state.d ./Libraries/coreMQTT/source/core_mqtt_state.o ./Libraries/coreMQTT/source/core_mqtt_state.su

.PHONY: clean-Libraries-2f-coreMQTT-2f-source

