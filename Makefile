# Inverted-Pendulum -- multi-target firmware build.
#
# Usage:
#   make BOARD=uno build          # compile firmware for a board
#   make BOARD=bluepill flash     # build + flash (OpenOCD for STM32)
#   make BOARD=esp32 monitor      # open a serial monitor
#   make BOARD=uno clean
#   make test                     # host-side unit tests (no hardware, no BOARD)
#   make list-boards
#
# Boards: uno, mega, esp32, bluepill. Each maps to a fragment in boards/<id>.mk
# which selects a toolchain family (avr/esp32/stm32) in make/<family>.mk.
#
# Toolchains are NOT bundled. Run scripts/install_toolchains.sh once to fetch
# avr-gcc, arm-none-eabi-gcc, xtensa-esp32-elf, avrdude, esptool and the Arduino
# core sources. OpenOCD is expected to be on PATH.

BOARDS := uno mega esp32 bluepill

.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "Inverted-Pendulum firmware"
	@echo ""
	@echo "  make BOARD=<id> build|flash|monitor|clean   (id: $(BOARDS))"
	@echo "  make test            run host unit tests (no hardware)"
	@echo "  make list-boards"
	@echo ""
	@echo "First-time setup: scripts/install_toolchains.sh"

.PHONY: list-boards
list-boards:
	@for b in $(BOARDS); do echo "  $$b"; done

# Host unit tests are board-independent; delegate to tests/Makefile.
.PHONY: test
test:
	$(MAKE) -C tests run

# Board-specific targets are only wired when BOARD is set. boards/<id>.mk sets
# FAMILY (+ MCU, defines, upload params); make/common.mk adds the shared source
# list; make/<family>.mk provides the build/flash/monitor recipes.
ifneq ($(BOARD),)
ifeq ($(filter $(BOARD),$(BOARDS)),)
$(error Unknown BOARD '$(BOARD)'. Valid: $(BOARDS))
endif
include boards/$(BOARD).mk
include make/common.mk
include make/$(FAMILY).mk
endif
