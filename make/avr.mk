# AVR family (Arduino Uno/Nano/Mega) -- avr-gcc toolchain, avrdude flashing.
#
# Expects the Arduino AVR core under $(ARDUINO_AVR_DIR), populated by
# scripts/install_toolchains.sh. Board fragments set MCU, F_CPU, VARIANT,
# AVRDUDE_PROGRAMMER and UPLOAD_* .

CC      := avr-gcc
CXX     := avr-g++
OBJCOPY := avr-objcopy
AR      := avr-gcc-ar
AVRDUDE := avrdude

ARDUINO_AVR_DIR ?= $(VENDOR_DIR)/arduino-avr
CORE_DIR    := $(ARDUINO_AVR_DIR)/cores/arduino
VARIANT_DIR := $(ARDUINO_AVR_DIR)/variants/$(VARIANT)

MCU_FLAGS := -mmcu=$(MCU) -DF_CPU=$(F_CPU)
INCLUDES  := $(PROJECT_INC) -I$(CORE_DIR) -I$(VARIANT_DIR) -I$(ACCELSTEPPER_DIR)
CPPFLAGS  := $(MCU_FLAGS) $(COMMON_DEFS) $(COMMON_FLAGS) $(INCLUDES)
CXXFLAGS  := $(COMMON_CXXFLAGS)

# Core + AccelStepper + project sources (the core ships .S assembly too).
CORE_SRC := $(wildcard $(CORE_DIR)/*.c) $(wildcard $(CORE_DIR)/*.cpp) \
            $(wildcard $(CORE_DIR)/*.S)
LIB_SRC  := $(wildcard $(ACCELSTEPPER_DIR)/*.cpp)
ALL_SRC  := $(CORE_SRC) $(LIB_SRC) $(PROJECT_SRC)
OBJS     := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(ALL_SRC)))

ELF := $(BUILD_DIR)/fw.elf
HEX := $(BUILD_DIR)/fw.hex

build: $(HEX)
	@echo "AVR build complete: $(HEX)"

$(BUILD_DIR)/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) -std=gnu11 -c $< -o $@

$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.S.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(MCU_FLAGS) $(COMMON_DEFS) -I$(CORE_DIR) -I$(VARIANT_DIR) -c $< -o $@

$(ELF): $(OBJS)
	$(CC) $(MCU_FLAGS) -Wl,--gc-sections -o $@ $(OBJS) -lm

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(HEX)
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMER) -p $(MCU) -P $(UPLOAD_PORT) \
		-b $(UPLOAD_BAUD) -D -U flash:w:$(HEX):i

monitor:
	@echo "Opening $(UPLOAD_PORT) @ $(UPLOAD_BAUD) (Ctrl-A k to quit screen)"
	screen $(UPLOAD_PORT) $(UPLOAD_BAUD)
