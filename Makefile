TARGET:=manufunkture

ROOT := $(CURDIR)/$(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# TODO change to your ARM gcc toolchain path
TOOLCHAIN_ROOT:=$(ROOT)/tools
ARM_TOOLCHAIN_PREFIX:=arm-none-eabi
ARM_TOOLCHAIN_ROOT:=$(TOOLCHAIN_ROOT)/gcc-$(ARM_TOOLCHAIN_PREFIX)
ARM_TOOLCHAIN_PATH:=$(ARM_TOOLCHAIN_ROOT)/bin

# STLINK toolchain path
STLINK_TOOLCHAIN_PREFIX:=st
STLINK_TOOLCHAIN_DIR:=stlink
STLINK_TOOLCHAIN_ROOT:=$(TOOLCHAIN_ROOT)/$(STLINK_TOOLCHAIN_DIR)
STLINK_TOOLCHAIN_PATH:=$(STLINK_TOOLCHAIN_ROOT)/bin

# Optimization level, can be [0, 1, 2, 3, s].
OPTLVL:=0
DBG:=-g

FREERTOS:=$(CURDIR)/libs/FreeRTOS/FreeRTOS/Source
STARTUP:=$(CURDIR)/hardware
LINKER_SCRIPT:=$(CURDIR)/linker/stm32_flash.ld

INCLUDE=-I$(CURDIR)/include
INCLUDE+=-I$(FREERTOS)/include
INCLUDE+=-I$(FREERTOS)/portable/GCC/ARM_CM4F
INCLUDE+=-I$(CURDIR)/libs/stm32f4xx-stdlib/Libraries/CMSIS/Device/ST/STM32F4xx/Include
INCLUDE+=-I$(CURDIR)/libs/stm32f4xx-stdlib/Libraries/CMSIS/Include
INCLUDE+=-I$(CURDIR)/libs/stm32f4xx-stdlib/Libraries/STM32F4xx_StdPeriph_Driver/inc
INCLUDE+=-I$(CURDIR)/config

BUILD_DIR = $(CURDIR)/build
BIN_DIR = $(CURDIR)/binary

# vpath is used so object files are written to the current directory instead
# of the same directory as their source files
vpath %.c $(CURDIR)/libs/stm32f4xx-stdlib/Libraries/STM32F4xx_StdPeriph_Driver/src \
          $(CURDIR)/libs/syscall $(CURDIR)/hardware $(FREERTOS) \
          $(FREERTOS)/portable/MemMang $(FREERTOS)/portable/GCC/ARM_CM4F 

vpath %.s $(STARTUP)
ASRC=startup_stm32f4xx.s

# Project Source Files
SRC+=stm32f4xx_it.c
SRC+=system_stm32f4xx.c
SRC+=main.c
SRC+=syscalls.c

# FreeRTOS Source Files
SRC+=port.c
SRC+=list.c
SRC+=queue.c
SRC+=tasks.c
SRC+=event_groups.c
SRC+=timers.c
SRC+=heap_4.c

# Standard Peripheral Source Files
SRC+=misc.c
SRC+=stm32f4xx_dcmi.c
#SRC+=stm32f4xx_hash.c
SRC+=stm32f4xx_rtc.c
SRC+=stm32f4xx_adc.c
SRC+=stm32f4xx_dma.c
#SRC+=stm32f4xx_hash_md5.c
SRC+=stm32f4xx_sai.c
SRC+=stm32f4xx_can.c
SRC+=stm32f4xx_dma2d.c
#SRC+=stm32f4xx_hash_sha1.c
SRC+=stm32f4xx_sdio.c
SRC+=stm32f4xx_cec.c
SRC+=stm32f4xx_dsi.c
SRC+=stm32f4xx_i2c.c
SRC+=stm32f4xx_spdifrx.c
SRC+=stm32f4xx_crc.c
SRC+=stm32f4xx_exti.c
SRC+=stm32f4xx_iwdg.c
SRC+=stm32f4xx_spi.c
#SRC+=stm32f4xx_cryp.c
SRC+=stm32f4xx_flash.c
SRC+=stm32f4xx_lptim.c
SRC+=stm32f4xx_syscfg.c
#SRC+=stm32f4xx_cryp_aes.c
SRC+=stm32f4xx_flash_ramfunc.c
SRC+=stm32f4xx_ltdc.c
SRC+=stm32f4xx_tim.c
#SRC+=stm32f4xx_cryp_des.c
#SRC+=stm32f4xx_fmc.c
SRC+=stm32f4xx_pwr.c
SRC+=stm32f4xx_usart.c
#SRC+=stm32f4xx_cryp_tdes.c
SRC+=stm32f4xx_fmpi2c.c
SRC+=stm32f4xx_qspi.c
SRC+=stm32f4xx_wwdg.c
SRC+=stm32f4xx_dac.c
SRC+=stm32f4xx_fsmc.c
SRC+=stm32f4xx_rcc.c
SRC+=stm32f4xx_dbgmcu.c
SRC+=stm32f4xx_gpio.c
SRC+=stm32f4xx_rng.c

CDEFS=-DUSE_STDPERIPH_DRIVER
CDEFS+=-DSTM32F4XX
CDEFS+=-DSTM32F40_41xxx
CDEFS+=-DHSE_VALUE=8000000
CDEFS+=-D__FPU_PRESENT=1
CDEFS+=-D__FPU_USED=1
CDEFS+=-DARM_MATH_CM4

MCUFLAGS=-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -finline-functions -Wdouble-promotion -std=gnu99
COMMONFLAGS=-O$(OPTLVL) $(DBG) -Wall -ffunction-sections -fdata-sections
CFLAGS=$(COMMONFLAGS) $(MCUFLAGS) $(INCLUDE) $(CDEFS)

LDLIBS=-lm -lc -lgcc
LDFLAGS=$(MCUFLAGS) -u _scanf_float -u _printf_float -fno-exceptions -Wl,--gc-sections,-T$(LINKER_SCRIPT),-Map,$(BIN_DIR)/$(TARGET).map

CC=$(ARM_TOOLCHAIN_PATH)/$(ARM_TOOLCHAIN_PREFIX)-gcc
LD=$(ARM_TOOLCHAIN_PATH)/$(ARM_TOOLCHAIN_PREFIX)-gcc
OBJCOPY=$(ARM_TOOLCHAIN_PATH)/$(ARM_TOOLCHAIN_PREFIX)-objcopy
AS=$(ARM_TOOLCHAIN_PATH)/$(ARM_TOOLCHAIN_PREFIX)-as
AR=$(ARM_TOOLCHAIN_PATH)/$(ARM_TOOLCHAIN_PREFIX)-ar
GDB=$(ARM_TOOLCHAIN_PATH)/$(ARM_TOOLCHAIN_PREFIX)-gdb
OBJDUMP=$(ARM_TOOLCHAIN_PATH)/$(ARM_TOOLCHAIN_PREFIX)-objdump
STFLASH=$(STLINK_TOOLCHAIN_PATH)/$(STLINK_TOOLCHAIN_PREFIX)-flash

OBJ = $(SRC:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c
	@echo [CC] $(notdir $<)
	@$(CC) $(CFLAGS) $< -c -o $@

all: $(OBJ)
	@echo [AS] $(ASRC)
	@$(AS) -o $(ASRC:%.s=$(BUILD_DIR)/%.o) $(STARTUP)/$(ASRC)
	@echo [LD] $(TARGET).elf
	@$(CC) -o $(BIN_DIR)/$(TARGET).elf $(LDFLAGS) $(OBJ) $(ASRC:%.s=$(BUILD_DIR)/%.o) $(LDLIBS)
	@echo [HEX] $(TARGET).hex
	@$(OBJCOPY) -O ihex $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex
	@echo [BIN] $(TARGET).bin
	@$(OBJCOPY) -O binary $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).bin

.PHONY: clean

clean:
	@echo [RM] OBJ
	@rm -f $(OBJ)
	@rm -f $(ASRC:%.s=$(BUILD_DIR)/%.o)
	@echo [RM] BIN
	@rm -f $(BIN_DIR)/$(TARGET).elf
	@rm -f $(BIN_DIR)/$(TARGET).hex
	@rm -f $(BIN_DIR)/$(TARGET).bin

size: 
	@$(SIZE) $(BIN_DIR)/$(TARGET).elf

disass: 
	@$(OBJDUMP) -d $(BIN_DIR)/$(TARGET).elf

disass-all: 
	@$(OBJDUMP) -D $(BIN_DIR)/$(TARGET).elf

debug:
	@$(GDB) --eval-command="target extended-remote :4242" \
	 $(BIN_DIR)/$(TARGET).elf

burn:
	@$(STFLASH) write $(BIN_DIR)/$(TARGET).bin 0x8000000
