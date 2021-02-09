################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/microshell/core/microshell.c \
../source/microshell/core/mscore.c 

OBJS += \
./source/microshell/core/microshell.o \
./source/microshell/core/mscore.o 

C_DEPS += \
./source/microshell/core/microshell.d \
./source/microshell/core/mscore.d 


# Each subdirectory must supply rules for building sources it contributes
source/microshell/core/%.o: ../source/microshell/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Nios II GCC C Compiler'
	nios2-elf-gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


