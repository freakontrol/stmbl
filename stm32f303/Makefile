# Optimization level, can be [0, 1, 2, 3, s].
#     0 = turn off optimization. s = optimize for size.
#
OPT = -O3
# OPT = -O1         # for debugging

# Object files directory
# Warning: this will be removed by make clean!
#
OBJDIR = obj_hvf3

# Target file name (without extension)
TARGET = $(OBJDIR)/hvf3

# Define all C source files (dependencies are generated automatically)
INCDIRS += stm32f303/inc
INCDIRS += stm32f303/inc/comps
INCDIRS += stm32f303/inc/shared_comps
SOURCES += stm32f303/src/main.c
SOURCES += stm32f303/src/adc.c
SOURCES += stm32f303/src/dac.c
SOURCES += stm32f303/src/opamp.c
SOURCES += stm32f303/src/stm32f3xx_hal_msp.c
SOURCES += stm32f303/src/tim.c
SOURCES += stm32f303/src/stm32f3xx_it.c
SOURCES += stm32f303/src/usbd_cdc_if.c
SOURCES += stm32f303/src/version.c

SOURCES += stm32f303/src/hal_tbl.c

CFLAGS += -DHAL_MAX_CTX=1024

include stm32f303/comps.mak

COMPS = $(F3COMPS)
SOURCES += $(COMPS)

INCDIRS += shared
SOURCES += shared/ringbuf.c
SOURCES += shared/crc8.c
SOURCES += shared/angle.c
SOURCES += shared/hal.c
SOURCES += shared/commands.c

#CMSIS
CPPFLAGS += -DSTM32F303xC
INCDIRS += lib/CMSIS/Include/
INCDIRS += lib/CMSIS/Device/ST/STM32F3xx/Include/
SOURCES += lib/CMSIS/Device/ST/STM32F3xx/Source/Templates/gcc/startup_stm32f303xc.s
SOURCES += lib/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c

#stm32f3 HAL Driver
HAL_DRV_DIR = lib/STM32F3xx_HAL_Driver/

INCDIRS += $(HAL_DRV_DIR)/Inc
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_adc.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_adc_ex.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_cortex.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_crc.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_crc_ex.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_dac.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_dac_ex.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_gpio.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_opamp.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_opamp_ex.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_rcc.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_rcc_ex.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_tim.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_tim_ex.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_uart.c
SOURCES += $(HAL_DRV_DIR)/Src/stm32f3xx_hal_uart_ex.c

LDSCRIPT = stm32f303/STM32F303CBTx_FLASH.ld

#============================================================================
SRC_COMP_OBJECTS = $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(HV_SRC_COMPS))))
SHARED_COMP_OBJECTS = $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(HV_SHARED_COMPS))))
OBJECTS += $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SOURCES))))
CPPFLAGS += $(addprefix -I,$(INCDIRS))

#---------------- Preprocessor Options ----------------
#  -fsingle...    make better use of the single-precision FPU
#  -g             generate debugging information
#  -save-temps    preserve .s and .i-files
#
#CPPFLAGS +=
# CPPFLAGS += -g
# CPPFLAGS += -save-temps=obj

#---------------- C Compiler Options ----------------
#  -O*            optimization level
#  -f...          tuning, see GCC documentation
#  -Wall...       warning level
#
CFLAGS += $(OPT)
CFLAGS += -std=gnu11
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wall
CFLAGS += -fno-builtin ## from old
CFLAGS += -nostartfiles
CFLAGS += -Wfatal-errors
CFLAGS += -Wno-pointer-sign #for usb lib...
#CFLAGS += -Wdouble-promotion
CFLAGS += -Wfloat-conversion
CFLAGS += -fsingle-precision-constant
# CFLAGS += -ffast-math
CFLAGS += -ffinite-math-only
CFLAGS += -fno-trapping-math
CFLAGS += -fno-signaling-nans
CFLAGS += -fno-rounding-math
CFLAGS += -fno-signed-zeros
CFLAGS += -fno-math-errno
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Wextra
#CFLAGS += -Wpointer-arith
#CFLAGS += -Winline
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wundef

# Use a friendly C dialect
CPPFLAGS += -fno-strict-aliasing
CPPFLAGS += -fwrapv

#---------------- C++ Compiler Options ----------------
#
CXXFLAGS += $(OPT)
CXXFLAGS += -ffunction-sections
CXXFLAGS += -fdata-sections
CXXFLAGS += -Wall

#---------------- Assembler Options ----------------
#  -Wa,...    tell GCC to pass this to the assembler
#

#---------------- Linker Options ----------------
#  -Wl,...      tell GCC to pass this to linker
#  -Map         create map file
#  --cref       add cross reference to  map file
#
LDFLAGS += $(OPT)
LDFLAGS += -lm
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections

# LDFLAGS += -specs=nano.specs -u _printf_float -u _scanf_float
LDFLAGS += -lc -specs=nosys.specs #fixes sbrk missing? present in eclipse?
LDFLAGS += -T$(LDSCRIPT)

#============================================================================

POSTLD   = $(PYTHON) tools/add_version_info.py # -q

# Compiler flags to generate dependency files
#
GENDEPFLAGS = -MMD -MP

# Combine all necessary flags and optional flags
# Add target processor to flags.
#
CPU = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS   += $(CPU)
CXXFLAGS += $(CPU)
ASFLAGS  += $(CPU)
LDFLAGS  += $(CPU)

ADDRESS = 0x8004000

# Default target
#
all: gccversion build showsize

hv_firmware.o: obj_hvf3/hvf3.bin
	$(OBJCOPY) --rename-section .data=.hv_firmware -I binary obj_hvf3/hvf3.bin -B arm -O elf32-littlearm hv_firmware.o

build: tbl elf hex bin lss sym hv_firmware.o

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: tbl $(TARGET).bin
lss: $(TARGET).lss
sym: $(TARGET).sym

# Display compiler version information
$(OBJDIR)/shared/commands.o: stm32f303/inc/commandslist.h

stm32f303/inc/commandslist.h: tools/create_cmd.py $(SOURCES)
	@echo Generating commands list
	@$(MKDIR) -p $(dir $@)
	@$(PYTHON) tools/create_cmd.py $@ $(SOURCES)

stm32f303/src/hal_tbl.c: tools/create_hal_tbl.py $(COMPS)
	@echo Generating HAL table
	@$(MKDIR) -p $(dir $@)
	@$(PYTHON) tools/create_hal_tbl.py $@ $(COMPS)

$(SRC_COMP_OBJECTS): $(OBJDIR)/stm32f303/src/comps/%.o: stm32f303/inc/comps/%_comp.h

$(SHARED_COMP_OBJECTS): $(OBJDIR)/shared/comps/%.o: stm32f303/inc/shared_comps/%_comp.h

stm32f303/inc/comps/%_comp.h: stm32f303/src/comps/%.c
	@echo Generating H: $<
	@$(MKDIR) -p $(dir $@)
	@$(PYTHON) tools/create_comp_h.py $@ $<

stm32f303/inc/shared_comps/%_comp.h: shared/comps/%.c
	@echo Generating H: $<
	@$(MKDIR) -p $(dir $@)
	@$(PYTHON) tools/create_comp_h.py $@ $<

tbl: stm32f303/src/hal_tbl.c stm32f303/inc/commandslist.h

# Target: clean project
#
clean:
	@echo Cleaning project:
	rm -rf $(OBJDIR)
	rm -rf stm32f303/inc/commandslist.h
	rm -rf stm32f303/inc/comps/*
	rm -rf stm32f303/inc/shared_comps/*
	rm -rf stm32f303/src/hal_tbl.c
	rm -rf hv_firmware.o

# Include the base rules
#
include base.mak

# Include the dependency files
#
-include $(OBJECTS:.o=.d)

# Listing of phony targets
#
.PHONY: all build clean \
        elf hex bin lss sym tbl
