################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Device/Driver/Src/r_dtc/r_dtc_api.c 

C_DEPS += \
./Device/Driver/Src/r_dtc/r_dtc_api.d 

OBJS += \
./Device/Driver/Src/r_dtc/r_dtc_api.o 


# Each subdirectory must supply rules for building sources it contributes
Device/Driver/Src/r_dtc/%.o: ../Device/Driver/Src/r_dtc/%.c
	arm-none-eabi-gcc -mcpu=cortex-m0plus -march=armv6-m -mthumb -mlittle-endian -mfloat-abi=soft -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wnull-dereference -g -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\CMSIS_Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\Device\Config" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\Core\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\Driver\Include" -I"C:\Workspace_e2studio\CMSIS_PACK_256KB_Rev100\RE01_256KB_DFP\CMSIS\DSP_Lib\Include" -IC:/Workspace_e2studio/CMSIS_PACK_256KB_Rev100/RE01_256KB_DFP/generate -IC:/Workspace_e2studio/CMSIS_PACK_256KB_Rev100/RE01_256KB_DFP/src -std=gnu11 -fno-jump-tables -fno-jump-tables -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

