################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/WIFI/es_wifi.c \
../Libraries/WIFI/es_wifi_io.c \
../Libraries/WIFI/wifi.c 

OBJS += \
./Libraries/WIFI/es_wifi.o \
./Libraries/WIFI/es_wifi_io.o \
./Libraries/WIFI/wifi.o 

C_DEPS += \
./Libraries/WIFI/es_wifi.d \
./Libraries/WIFI/es_wifi_io.d \
./Libraries/WIFI/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/WIFI/%.o Libraries/WIFI/%.su Libraries/WIFI/%.cyclo: ../Libraries/WIFI/%.c Libraries/WIFI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"C:/Users/rober/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_notify/L4_IOT_Sensors/Drivers/BSP/Components/lsm6dsl" -I"C:/Users/rober/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_notify/L4_IOT_Sensors/Drivers/BSP/B-L475E-IOT01" -I"C:/Users/rober/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_notify/L4_IOT_Sensors/Drivers/BSP/Components/hts221" -I"C:/Users/rober/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_notify/Libraries/WIFI" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/rober/STM32CubeIDE/workspace_1.13.2/TrabajoSCF_notify/Libraries/coreMQTT/source" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-WIFI

clean-Libraries-2f-WIFI:
	-$(RM) ./Libraries/WIFI/es_wifi.cyclo ./Libraries/WIFI/es_wifi.d ./Libraries/WIFI/es_wifi.o ./Libraries/WIFI/es_wifi.su ./Libraries/WIFI/es_wifi_io.cyclo ./Libraries/WIFI/es_wifi_io.d ./Libraries/WIFI/es_wifi_io.o ./Libraries/WIFI/es_wifi_io.su ./Libraries/WIFI/wifi.cyclo ./Libraries/WIFI/wifi.d ./Libraries/WIFI/wifi.o ./Libraries/WIFI/wifi.su

.PHONY: clean-Libraries-2f-WIFI

