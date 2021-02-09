################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../source/driver/adc2.cpp \
../source/driver/imu.cpp 

OBJS += \
./source/driver/adc2.o \
./source/driver/imu.o 

CPP_DEPS += \
./source/driver/adc2.d \
./source/driver/imu.d 


# Each subdirectory must supply rules for building sources it contributes
source/driver/%.o: ../source/driver/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Nios II GCC C++ Compiler'
	nios2-elf-g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


