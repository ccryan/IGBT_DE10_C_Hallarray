################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := clean

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: DE10_SoC_HallArray

# Tool invocations
DE10_SoC_HallArray: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C Linker 4.9.4 [arm-linux-gnueabihf]'
	arm-linux-gnueabihf-gcc.exe -L"C:\working\gcc-linaro-4.9.4-2017.01-i686-mingw32_arm-linux-gnueabihf\lib" -o "DE10_SoC_HallArray" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES) $(OBJS) $(C_DEPS) DE10_SoC_HallArray 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
