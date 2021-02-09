################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../source/centralized_monitor.cpp \
../source/main.cpp \
../source/shared_memory.cpp \
../source/stream_transmitter.cpp 

OBJS += \
./source/centralized_monitor.o \
./source/main.o \
./source/shared_memory.o \
./source/stream_transmitter.o 

CPP_DEPS += \
./source/centralized_monitor.d \
./source/main.d \
./source/shared_memory.d \
./source/stream_transmitter.d 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Nios II GCC C++ Compiler'
	nios2-elf-g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


