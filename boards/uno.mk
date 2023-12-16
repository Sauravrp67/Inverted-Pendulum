# Arduino Uno / Nano -- ATmega328P @ 16 MHz.
FAMILY    := avr
MCU       := atmega328p
F_CPU     := 16000000L
VARIANT   := standard
BOARD_DEF := BOARD_UNO

AVRDUDE_PROGRAMMER := arduino
UPLOAD_PORT ?= /dev/ttyUSB0
UPLOAD_BAUD := 115200
