# The IDF `main` component: compiles the shared firmware + libraries against the
# Arduino API provided by the `arduino` component. No app_main() here -- with
# CONFIG_AUTOSTART_ARDUINO the arduino component starts setup()/loop() for us.

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

idf_component_register(
  SRCS
    "${REPO_ROOT}/src/main.cpp"
    "${REPO_ROOT}/lib/PIDController/PIDController.cpp"
    "${REPO_ROOT}/lib/QuadratureEncoder/QuadratureEncoder.cpp"
    "${REPO_ROOT}/lib/StepperDriver/StepperDriver.cpp"
    "${REPO_ROOT}/vendor/AccelStepper/src/AccelStepper.cpp"
  INCLUDE_DIRS
    "${REPO_ROOT}/lib/PIDController"
    "${REPO_ROOT}/lib/QuadratureEncoder"
    "${REPO_ROOT}/lib/StepperDriver"
    "${REPO_ROOT}/lib/BoardConfig"
    "${REPO_ROOT}/src"
    "${REPO_ROOT}/vendor/AccelStepper/src"
  REQUIRES arduino)

# Select the ESP32 pin map (BoardConfig.h) and satisfy the #ifdef ARDUINO guards
# in QuadratureEncoder/StepperDriver, which IDF does not define by default.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
  BOARD_ESP32
  ARDUINO=10819
  ARDUINO_ARCH_ESP32)
