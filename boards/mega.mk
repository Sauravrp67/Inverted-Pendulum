# Arduino Mega 2560 -- ATmega2560 @ 16 MHz.
FAMILY    := avr
MCU       := atmega2560
F_CPU     := 16000000L
VARIANT   := mega
BOARD_DEF := BOARD_MEGA

AVRDUDE_PROGRAMMER := wiring
UPLOAD_PORT ?= /dev/ttyUSB0
UPLOAD_BAUD := 115200
