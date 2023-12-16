#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-release}"
DRY_RUN=false

if [[ "${2:-}" == "--dry-run" ]]; then
  DRY_RUN=true
elif [[ -n "${2:-}" ]]; then
  echo "usage: $0 [release] [--dry-run]" >&2
  exit 2
fi

if [[ "$PROFILE" != "release" ]]; then
  echo "unsupported build profile: $PROFILE (expected: release)" >&2
  exit 2
fi

cd "$REPO_ROOT"

if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  for toolchain_bin in \
    "${ARM_GNU_TOOLCHAIN_BIN:-}" \
    "$HOME"/toolchains/arm-gnu-toolchain-*-x86_64-arm-none-eabi/bin
  do
    if [[ -n "$toolchain_bin" && -x "$toolchain_bin/arm-none-eabi-g++" ]]; then
      export PATH="$toolchain_bin:$PATH"
      break
    fi
  done
fi

if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  echo "ARM GNU toolchain not found (missing arm-none-eabi-g++)" >&2
  echo "run scripts/install_toolchains.sh stm32 or set ARM_GNU_TOOLCHAIN_BIN" >&2
  exit 1
fi

echo "building Blue Pill release firmware"
make BOARD=bluepill build

ELF="build/bluepill/fw.elf"
BIN="build/bluepill/fw.bin"

if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  FLASH_CMD=(
    STM32_Programmer_CLI
    -c port=SWD
    -w "$BIN" 0x08000000
    -v
    -rst
  )
elif command -v openocd >/dev/null 2>&1; then
  FLASH_CMD=(
    openocd
    -f boards/bluepill/openocd.cfg
    -c "program $ELF verify reset exit"
  )
else
  echo "no supported flashing tool found" >&2
  echo "install STM32CubeProgrammer CLI or OpenOCD with ST-LINK support" >&2
  exit 1
fi

printf 'flash command:'
printf ' %q' "${FLASH_CMD[@]}"
printf '\n'

if "$DRY_RUN"; then
  exit 0
fi

"${FLASH_CMD[@]}"
