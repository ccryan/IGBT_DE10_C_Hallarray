################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hps_linux.c 

OBJS += \
./hps_linux.o 

C_DEPS += \
./hps_linux.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler 4.9.4 [arm-linux-gnueabihf]'
	arm-linux-gnueabihf-gcc.exe -Dsoc_cv_av -I"C:\intelFPGA\18.1\embedded\ip\altera\hps\altera_hps\hwlib\include\soc_cv_av" -I"C:\working\gcc-linaro-4.9.4-2017.01-i686-mingw32_arm-linux-gnueabihf\include" -I"C:\intelFPGA\18.1\embedded\ip\altera\hps\altera_hps\hwlib\include" -I"C:\MinGW\msys\1.0\local\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


