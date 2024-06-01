#pragma once
#include <Arduino.h>

namespace pins {
#if defined(__AVR_ATmega2560__)
constexpr int pole_a = 2, pole_b = 3, arm_a = 18, arm_b = 19;
constexpr int step_pin = 9, dir_pin = 8;
#elif defined(ARDUINO_ARCH_STM32)
constexpr int pole_a = PB6, pole_b = PB7, arm_a = PA0, arm_b = PA1;
constexpr int step_pin = PA4, dir_pin = PA5; // leave USART1 free
#else
#error "Supported boards: Mega 2560 and STM32 Blue Pill."
#endif

} // namespace pins
