################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SPI_communication.c \
../arithmetic.c \
../eeprom.c \
../flash.c \
../main.c \
../measurements.c \
../n1110.c \
../new.c \
../system_stm32f10x.c \
../uart.c 

S_UPPER_SRCS += \
../startup_stm32f10x_ld_vl.S 

OBJS += \
./SPI_communication.o \
./arithmetic.o \
./eeprom.o \
./flash.o \
./main.o \
./measurements.o \
./n1110.o \
./new.o \
./startup_stm32f10x_ld_vl.o \
./system_stm32f10x.o \
./uart.o 

S_UPPER_DEPS += \
./startup_stm32f10x_ld_vl.d 

C_DEPS += \
./SPI_communication.d \
./arithmetic.d \
./eeprom.d \
./flash.d \
./main.d \
./measurements.d \
./n1110.d \
./new.d \
./system_stm32f10x.d \
./uart.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -march=armv7-m -mthumb -mlittle-endian -munaligned-access -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -fsingle-precision-constant -flto -Wall -DSTM32F10X_LD_VL -DN1110 -DVER="\" RLC v  6.20b4 \"" -I"D:\Ivan\Projects\RLC-meter\rlcmeter_620b12_src\StdPeriph_stm32F1xx" -I"D:\Ivan\Projects\RLC-meter\rlcmeter_620b12_src\StdPeriph_stm32F1xx\inc" -I"D:\Ivan\Projects\RLC-meter\rlcmeter_620b12_src\rlcmeter2_5_bench_stm32F100" -std=gnu99 --fast-math -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -march=armv7-m -mthumb -mlittle-endian -munaligned-access -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -fsingle-precision-constant -flto -Wall -x assembler-with-cpp -DSTM32F10X_LD_VL -I"D:\Ivan\Projects\RLC-meter\rlcmeter_620b12_src\StdPeriph_stm32F1xx" -I"D:\Ivan\Projects\RLC-meter\rlcmeter_620b12_src\StdPeriph_stm32F1xx\inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


