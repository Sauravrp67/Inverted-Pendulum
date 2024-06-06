#pragma once

#ifdef __AVR__
#include <avr/pgmspace.h>
#define FIRMWARE_FLASH PROGMEM

static inline float coefficient(const float *p) { return pgm_read_float(p); }
#else
#define FIRMWARE_FLASH

static inline float coefficient(const float *p) { return *p; }
#endif
