################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Device/CMSIS_Driver/Src/r_i2c_cmsis_api.c \
../Device/CMSIS_Driver/Src/r_spi_cmsis_api.c \
../Device/CMSIS_Driver/Src/r_usart_cmsis_api.c 

C_DEPS += \
./Device/CMSIS_Driver/Src/r_i2c_cmsis_api.d \
./Device/CMSIS_Driver/Src/r_spi_cmsis_api.d \
./Device/CMSIS_Driver/Src/r_usart_cmsis_api.d 

OBJS += \
./Device/CMSIS_Driver/Src/r_i2c_cmsis_api.o \
./Device/CMSIS_Driver/Src/r_spi_cmsis_api.o \
./Device/CMSIS_Driver/Src/r_usart_cmsis_api.o 


# Each subdirectory must supply rules for building sources it contributes
Device/CMSIS_Driver/Src/%.o: ../Device/CMSIS_Driver/Src/%.c
	arm-none-eabi-gcc -mcpu=cortex-m0plus -march=armv6-m -mthumb -mlittle-endian -mfloat-abi=soft -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wnull-dereference -g -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\CMSIS_Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\Config" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\Core\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\DSP_Lib\Include" -IC:/Workspace_e2studio/CMSIS_PACK_256KB_Rev100/RE01_256KB_DFP/generate -IC:/Workspace_e2studio/CMSIS_PACK_256KB_Rev100/RE01_256KB_DFP/src -std=gnu11 -fno-jump-tables -fno-jump-tables -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

