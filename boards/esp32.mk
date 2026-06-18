# ESP32 dev module. Built via ESP-IDF (see make/esp32.mk + boards/esp32/).
FAMILY    := esp32
BOARD_DEF := BOARD_ESP32

# Serial port for idf.py flash/monitor. 460800 is a safe default upload baud.
UPLOAD_PORT ?= /dev/ttyUSB0
UPLOAD_BAUD := 460800
