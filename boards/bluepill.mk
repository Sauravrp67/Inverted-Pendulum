# STM32 Blue Pill -- STM32F103C8 (Cortex-M3), 64KB flash / 20KB RAM.
FAMILY    := stm32
BOARD_DEF := BOARD_BLUEPILL

MCU_FLAGS  := -mcpu=cortex-m3 -mthumb
STM32_DEFS := -DSTM32F1xx -DSTM32F103xB -DARDUINO_ARCH_STM32 \
              -DARDUINO_BLUEPILL_F103C8 -DF_CPU=72000000L

# Linker script and OpenOCD configs live alongside this fragment.
LDSCRIPT    := boards/bluepill/STM32F103C8Tx_FLASH.ld
OPENOCD_CFG := -f boards/bluepill/openocd.cfg

# stm32duino CMSIS/series include paths (resolved under ARDUINO_STM32_DIR).
SERIES_INC ?=

UPLOAD_PORT ?= /dev/ttyACM0
UPLOAD_BAUD := 115200
