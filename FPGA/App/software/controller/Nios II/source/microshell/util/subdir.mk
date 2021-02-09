################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/microshell/util/mscmd.c \
../source/microshell/util/msopt.c \
../source/microshell/util/ntlibc.c 

OBJS += \
./source/microshell/util/mscmd.o \
./source/microshell/util/msopt.o \
./source/microshell/util/ntlibc.o 

C_DEPS += \
./source/microshell/util/mscmd.d \
./source/microshell/util/msopt.d \
./source/microshell/util/ntlibc.d 


# Each subdirectory must supply rules for building sources it contributes
source/microshell/util/%.o: ../source/microshell/util/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Nios II GCC C Compiler'
	nios2-elf-gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


