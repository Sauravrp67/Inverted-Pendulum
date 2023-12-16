# ESP32 family -- ESP-IDF build, wrapped by make.
#
# Unlike AVR/STM32 (whose Arduino "core" is a flat pile of .c/.cpp/.S we compile
# directly), arduino-esp32 is built on ESP-IDF: FreeRTOS + hundreds of
# components + a generated sdkconfig + a multi-stage link. The native, reliable
# way to build it from the raw xtensa toolchain is ESP-IDF's own CMake/ninja
# build, driven by idf.py. We therefore keep a small IDF project under
# boards/esp32/ (with arduino-esp32 as a component) and just invoke idf.py here.
#
# Prerequisites (see scripts/install_toolchains.sh esp32):
#   * ESP-IDF installed and its environment sourced:  . $IDF_PATH/export.sh
#     (this puts idf.py + xtensa-esp32-elf on PATH)
#   * vendor/arduino-esp32 cloned (the Arduino component)
# The same src/main.cpp + lib/* run here: CONFIG_AUTOSTART_ARDUINO starts
# setup()/loop() (see boards/esp32/sdkconfig.defaults).

IDF_PROJECT := boards/esp32
IDF         := idf.py

# Keep the IDF build tree under build/esp32 like the other boards.
IDF_ARGS := -C $(IDF_PROJECT) -B $(BUILD_DIR)

build:
	@command -v $(IDF) >/dev/null 2>&1 || { \
		echo "idf.py not found. Install ESP-IDF and run '. \$$IDF_PATH/export.sh' first"; \
		echo "(scripts/install_toolchains.sh esp32)"; exit 1; }
	@test -d $(IDF_PROJECT)/components/arduino || { \
		echo "arduino-esp32 component missing (scripts/install_toolchains.sh cores)"; exit 1; }
	$(IDF) $(IDF_ARGS) build
	@echo "ESP32 build complete: $(BUILD_DIR)"

flash:
	$(IDF) $(IDF_ARGS) -p $(UPLOAD_PORT) -b $(UPLOAD_BAUD) flash

monitor:
	$(IDF) $(IDF_ARGS) -p $(UPLOAD_PORT) monitor
