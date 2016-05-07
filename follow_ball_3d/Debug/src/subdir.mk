################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Actue_Node.cpp \
../src/Percept_Node.cpp \
../src/Scan_Node.cpp 

OBJS += \
./src/Actue_Node.o \
./src/Percept_Node.o \
./src/Scan_Node.o 

CPP_DEPS += \
./src/Actue_Node.d \
./src/Percept_Node.d \
./src/Scan_Node.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/pcl-1.7 -I/home/javier/git/CSUROS/csuro_tools/include -I/usr/include/gtkmm-2.4/ -I/usr/include/libglademm-2.4 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


