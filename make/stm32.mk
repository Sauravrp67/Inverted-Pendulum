# STM32 family (Blue Pill / F103) -- arm-none-eabi toolchain, OpenOCD flashing.
#
# This is the primary arm-none-eabi + OpenOCD target. Uses the Arduino_Core_STM32
# (stm32duino) sources under $(ARDUINO_STM32_DIR), populated by
# scripts/install_toolchains.sh. Board fragment (boards/bluepill.mk) sets MCU
# flags, the STM32 product define, the linker script and the OpenOCD configs.

CROSS   := arm-none-eabi-
CC      := $(CROSS)gcc
CXX     := $(CROSS)g++
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size
OPENOCD := openocd

ARDUINO_STM32_DIR ?= $(VENDOR_DIR)/arduino-stm32
CORE_DIR    := $(ARDUINO_STM32_DIR)/cores/arduino
CMSIS_DIR   := $(ARDUINO_STM32_DIR)/system/Drivers/CMSIS

# MCU_FLAGS, STM32_DEFS, LDSCRIPT, OPENOCD_CFG and SERIES_INC come from the board.
INCLUDES := $(PROJECT_INC) -I$(CORE_DIR) $(SERIES_INC) -I$(ACCELSTEPPER_DIR)
CPPFLAGS := $(MCU_FLAGS) $(STM32_DEFS) $(COMMON_DEFS) $(COMMON_FLAGS) $(INCLUDES)
CXXFLAGS := $(COMMON_CXXFLAGS)
LDFLAGS  := $(MCU_FLAGS) -T$(LDSCRIPT) -Wl,--gc-sections \
            -Wl,--defsym=LD_FLASH_OFFSET=0 --specs=nano.specs -lc -lm

CORE_SRC := $(wildcard $(CORE_DIR)/*.c) $(wildcard $(CORE_DIR)/*.cpp) \
            $(wildcard $(CORE_DIR)/*.S)
LIB_SRC  := $(wildcard $(ACCELSTEPPER_DIR)/*.cpp)
ALL_SRC  := $(CORE_SRC) $(LIB_SRC) $(PROJECT_SRC)
OBJS     := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(ALL_SRC)))

ELF := $(BUILD_DIR)/fw.elf
BIN := $(BUILD_DIR)/fw.bin

build: $(BIN)
	@echo "STM32 build complete: $(ELF)"
	@$(SIZE) $(ELF)

$(BUILD_DIR)/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) -std=gnu11 -c $< -o $@

$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.S.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) -c $< -o $@

$(ELF): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $(OBJS)

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

# Flash the .elf via OpenOCD + ST-Link. program ... verify reset exit is the
# canonical one-shot recipe.
flash: $(ELF)
	$(OPENOCD) $(OPENOCD_CFG) \
		-c "program $(ELF) verify reset exit"

monitor:
	@echo "Opening $(UPLOAD_PORT) @ $(UPLOAD_BAUD) (Ctrl-A k to quit screen)"
	screen $(UPLOAD_PORT) $(UPLOAD_BAUD)
