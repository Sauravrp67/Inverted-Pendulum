# Shared definitions for all board builds. Included after boards/<id>.mk (which
# sets FAMILY/BOARD_DEF/...) and before make/<family>.mk (which sets the
# compiler and recipes). All paths are overridable from the environment so the
# install script / CI can relocate the vendored sources.

BUILD_DIR := build/$(BOARD)

# Project (non-vendor) sources: the shared libraries + one firmware entrypoint.
# SRC_DIR defaults to src/ (the balancer) but can point at an examples/ dir.
SRC_DIR     ?= src
LIB_DIRS    := lib/PIDController lib/QuadratureEncoder lib/StepperDriver lib/BoardConfig
PROJECT_SRC := $(wildcard lib/*/*.cpp) $(wildcard $(SRC_DIR)/*.cpp)
PROJECT_INC := $(addprefix -I,$(LIB_DIRS)) -I$(SRC_DIR)

# Vendored third-party sources fetched by scripts/install_toolchains.sh.
VENDOR_DIR       ?= vendor
ACCELSTEPPER_DIR ?= $(VENDOR_DIR)/AccelStepper/src

# Common preprocessor defines. ARDUINO marks the Arduino-API build (guards the
# host-only code paths); BOARD_DEF (e.g. BOARD_UNO) selects the pin map.
COMMON_DEFS := -DARDUINO=10819 -D$(BOARD_DEF)

# Common warning/optimisation flags shared by every embedded toolchain.
COMMON_FLAGS := -Os -ffunction-sections -fdata-sections -Wall -Wextra
COMMON_CXXFLAGS := -std=gnu++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics

.PHONY: build flash monitor clean
clean:
	rm -rf $(BUILD_DIR)
