C_SRC := \
$(wildcard ../src/*.c) \
$(wildcard ../src/modules/dc_motor/*.c) \
$(wildcard ../src/modules/encoder/*.c) \
$(wildcard ../src/modules/odometry/*.c) \
$(wildcard ../src/tasks/hc_sr04_survey/*.c) \
$(wildcard ../src/tasks/hw_201_survey/*.c) \
$(wildcard ../src/tasks/power_measure/*.c) \
$(wildcard ../src/tasks/cmd_interface/*.c) \
$(wildcard ../src/tasks/motion_control/*.c) \
$(wildcard ../lib/m_string/*.c) \
$(wildcard ../lib/hc_sr04/*.c) \
$(wildcard ../lib/hw_201/*.c) \
$(wildcard ../lib/ina226/*.c) \
$(wildcard ../lib/w25q32/*.c) \
$(wildcard ../lib/FreeRTOS/*.c) \
$(wildcard ../lib/FreeRTOS/portable/GCC/RISC-V/*.c) \
$(wildcard ../lib/FreeRTOS/portable/MemMang/*.c) \
$(wildcard ../hardware/*.c) \
$(wildcard ../hardware/uart/*.c) \
$(wildcard ../hardware/core/*.c) \
$(wildcard ../hardware/gpio/*.c) \
$(wildcard ../hardware/i2c/*.c) \
$(wildcard ../hardware/misc/*.c) \
$(wildcard ../hardware/rcc/*.c) \
$(wildcard ../hardware/spi/*.c) \
$(wildcard ../hardware/tim/*.c) \

S_SRC := \
$(wildcard ../hardware/startup/*.S) \
$(wildcard ../lib/FreeRTOS/portable/GCC/RISC-V/*.S) \

OBJDIR := ./obj
OBJ := $(C_SRC:.c=.o)
OBJ_S := $(S_SRC:.S=.o)

# DEPS := $(wildcard $(OBJDIR)/*.d)

INC_DIRS := \
../hardware \
../hardware/core \
../hardware/gpio \
../hardware/i2c \
../hardware/misc \
../hardware/rcc \
../hardware/spi \
../hardware/tim \
../hardware/uart \
../lib/FreeRTOS/include \
../lib/FreeRTOS/portable/GCC/RISC-V \
../lib/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions \
../lib/m_string \
../lib/w25q32 \
../lib/ina226 \
../lib/hc_sr04 \
../lib/hw_201 \
../src \
../src/modules/dc_motor \
../src/modules/encoder \
../src/modules/odometry \
../src/tasks/cmd_interface \
../src/tasks/motion_control \
../src/tasks/hw_201_survey \
../src/tasks/hc_sr04_survey \
../src/tasks/power_measure \

LINKER_SCRIPT := ../hardware/linker/Link.ld

