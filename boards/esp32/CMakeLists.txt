# ESP-IDF top-level project for the ESP32 build of the inverted-pendulum firmware.
#
# arduino-esp32 is pulled in as an extra component from vendor/. The firmware
# itself (src/main.cpp + lib/*) is compiled by the `main` component
# (see main/CMakeLists.txt). Build/flash via the repo Makefile:
#   make BOARD=esp32 build|flash|monitor
cmake_minimum_required(VERSION 3.16)

# arduino-esp32 is auto-discovered as the `arduino` component via the symlink
# components/arduino -> ../../../vendor/arduino-esp32 (the component name must be
# `arduino` so `REQUIRES arduino` resolves; IDF names components by directory).
# scripts/install_toolchains.sh creates that symlink and clones the core.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(inverted_pendulum)
