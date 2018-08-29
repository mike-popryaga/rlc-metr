################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
rlcmeter2_5_bench_stm32F100.hex \

SECONDARY_LIST += \
rlcmeter2_5_bench_stm32F100.lst \

SECONDARY_SIZE += \
rlcmeter2_5_bench_stm32F100.siz \


# All Target
all: rlcmeter2_5_bench_stm32F100.elf secondary-outputs

dependents:
	-cd D:/Ivan/Projects/RLC-meter/rlcmeter_620b12_src/StdPeriph_stm32F1xx/Release && $(MAKE) all

# Tool invocations
rlcmeter2_5_bench_stm32F100.elf: $(OBJS) $(USER_OBJS) D:/Ivan/Projects/RLC-meter/rlcmeter_620b12_src/StdPeriph_stm32F1xx/Release/libStdPeriph_stm32F1xx.a
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -march=armv7-m -mthumb -mlittle-endian -munaligned-access -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -fsingle-precision-constant -flto -Wall -T "D:\Ivan\Projects\RLC-meter\rlcmeter_620b12_src\rlcmeter2_5_bench_stm32F100\link.ld" -nostartfiles -Xlinker --gc-sections -s -L"D:\Ivan\Projects\RLC-meter\rlcmeter_620b12_src\StdPeriph_stm32F1xx\Release" -Wl,-Map,"rlcmeter2_5_bench_stm32F100.map" -specs=nosys.specs -specs=nano.specs -lc -lnosys -Wl,--gc-sections -o "rlcmeter2_5_bench_stm32F100.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

rlcmeter2_5_bench_stm32F100.hex: rlcmeter2_5_bench_stm32F100.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "rlcmeter2_5_bench_stm32F100.elf"  "rlcmeter2_5_bench_stm32F100.hex"
	@echo 'Finished building: $@'
	@echo ' '

rlcmeter2_5_bench_stm32F100.lst: rlcmeter2_5_bench_stm32F100.elf
	@echo 'Invoking: Cross ARM GNU Create Listing'
	arm-none-eabi-objdump --all-headers --demangle --disassemble --reloc --wide "rlcmeter2_5_bench_stm32F100.elf" > "rlcmeter2_5_bench_stm32F100.lst"
	@echo 'Finished building: $@'
	@echo ' '

rlcmeter2_5_bench_stm32F100.siz: rlcmeter2_5_bench_stm32F100.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley --totals "rlcmeter2_5_bench_stm32F100.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) rlcmeter2_5_bench_stm32F100.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:
D:/Ivan/Projects/RLC-meter/rlcmeter_620b12_src/StdPeriph_stm32F1xx/Release/libStdPeriph_stm32F1xx.a:

-include ../makefile.targets
