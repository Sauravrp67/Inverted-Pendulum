#!/usr/bin/env bash
# Install the embedded toolchains and vendored Arduino core sources needed to
# build this project for AVR, ESP32 and STM32. OpenOCD is assumed to be on PATH.
#
# Usage:
#   scripts/install_toolchains.sh [all|avr|stm32|esp32|cores]
#
# Designed for Debian/Ubuntu (apt). On other distros, install the equivalent
# packages and re-run only the `cores` step. Re-running is safe (idempotent).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_DIR="${VENDOR_DIR:-$REPO_ROOT/vendor}"
TARGET="${1:-all}"

log() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }

have() { command -v "$1" >/dev/null 2>&1; }

install_avr() {
  log "AVR toolchain (avr-gcc, avr-libc, avrdude)"
  if have apt-get; then
    sudo apt-get update -qq
    sudo apt-get install -y gcc-avr avr-libc avrdude
  else
    log "Non-apt system: install gcc-avr / avr-libc / avrdude via your package manager."
  fi
}

install_stm32() {
  log "STM32 toolchain (arm-none-eabi-gcc) + OpenOCD"
  if have apt-get; then
    sudo apt-get update -qq
    sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi openocd
  else
    log "Non-apt system: install gcc-arm-none-eabi + openocd via your package manager."
  fi
}

install_esp32() {
  # ESP32 is built via ESP-IDF (CMake/ninja + xtensa toolchain), with
  # arduino-esp32 as a component. This installs ESP-IDF itself; the
  # arduino-esp32 component is cloned by install_cores().
  IDF_DIR="${IDF_PATH:-$HOME/esp/esp-idf}"
  IDF_BRANCH="${IDF_BRANCH:-v5.1.4}"  # match the arduino-esp32 release you use
  log "ESP-IDF -> $IDF_DIR (branch $IDF_BRANCH)"
  if have apt-get; then
    sudo apt-get update -qq
    sudo apt-get install -y git wget flex bison gperf cmake ninja-build \
      ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
  fi
  if [ ! -d "$IDF_DIR/.git" ]; then
    git clone --recursive --branch "$IDF_BRANCH" \
      https://github.com/espressif/esp-idf.git "$IDF_DIR"
  fi
  "$IDF_DIR/install.sh" esp32
  log "ESP-IDF installed. Before building ESP32, source its environment:"
  log "  . $IDF_DIR/export.sh"
}

clone() {
  local url="$1" dir="$2" ref="${3:-}"
  if [ -d "$dir/.git" ]; then
    log "Updating $(basename "$dir")"
    git -C "$dir" fetch --depth 1 origin "${ref:-HEAD}" || true
  else
    log "Cloning $(basename "$dir")"
    git clone --depth 1 ${ref:+--branch "$ref"} "$url" "$dir"
  fi
}

install_cores() {
  log "Vendored Arduino cores + libraries -> $VENDOR_DIR"
  mkdir -p "$VENDOR_DIR"
  clone https://github.com/waspinator/AccelStepper.git "$VENDOR_DIR/AccelStepper"
  clone https://github.com/arduino/ArduinoCore-avr.git "$VENDOR_DIR/arduino-avr"
  clone https://github.com/stm32duino/Arduino_Core_STM32.git "$VENDOR_DIR/arduino-stm32"
  # arduino-esp32 is the IDF component used by boards/esp32/. Pin a release that
  # matches the installed ESP-IDF (see IDF_BRANCH in install_esp32).
  clone https://github.com/espressif/arduino-esp32.git "$VENDOR_DIR/arduino-esp32" 3.0.7
  # Ensure the components/arduino symlink IDF discovers exists (idempotent).
  ln -snf ../../../vendor/arduino-esp32 "$REPO_ROOT/boards/esp32/components/arduino"
}

case "$TARGET" in
  avr)   install_avr ;;
  stm32) install_stm32 ;;
  esp32) install_esp32 ;;
  cores) install_cores ;;
  all)   install_avr; install_stm32; install_esp32; install_cores ;;
  *) echo "Unknown target '$TARGET' (use: all|avr|stm32|esp32|cores)"; exit 1 ;;
esac

log "Done. Toolchain status:"
for t in avr-gcc avrdude arm-none-eabi-gcc openocd idf.py xtensa-esp32-elf-gcc; do
  printf '  %-22s %s\n' "$t" "$(command -v "$t" 2>/dev/null || echo '(missing)')"
done
