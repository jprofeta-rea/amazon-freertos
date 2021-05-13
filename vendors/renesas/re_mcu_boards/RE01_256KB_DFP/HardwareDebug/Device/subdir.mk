################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Device/pin.c \
../Device/startup_RE01_256KB.c \
../Device/system_RE01_256KB.c 

C_DEPS += \
./Device/pin.d \
./Device/startup_RE01_256KB.d \
./Device/system_RE01_256KB.d 

OBJS += \
./Device/pin.o \
./Device/startup_RE01_256KB.o \
./Device/system_RE01_256KB.o 


# Each subdirectory must supply rules for building sources it contributes
Device/%.o: ../Device/%.c
	arm-none-eabi-gcc -mcpu=cortex-m0plus -march=armv6-m -mthumb -mlittle-endian -mfloat-abi=soft -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wnull-dereference -g -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\CMSIS_Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\Config" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\Core\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\DSP_Lib\Include" -IC:/Workspace_e2studio/CMSIS_PACK_256KB_Rev100/RE01_256KB_DFP/generate -IC:/Workspace_e2studio/CMSIS_PACK_256KB_Rev100/RE01_256KB_DFP/src -std=gnu11 -fno-jump-tables -fno-jump-tables -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

