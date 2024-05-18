# Host tests: make test from the repository root.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O1 -g

CLASSICAL := ../controllers/classical
FIRMWARE := ../firmware
INCLUDES := -I. -I$(CLASSICAL) -I$(FIRMWARE)

# Test sources + the platform-independent library sources under test. The
# QuadratureDecoder is header-only; PIDController needs its .cpp.
SRCS := test_main.cpp test_pid.cpp test_encoder.cpp test_mpc.cpp \
        $(CLASSICAL)/PIDController.cpp ../controllers/mpc/mpc.cpp

BIN := build/run_tests

.PHONY: run build clean
run: build
	@./$(BIN)

build: $(BIN)

$(BIN): $(SRCS) $(CLASSICAL)/PIDController.h $(FIRMWARE)/quadrature_decoder.h ../controllers/mpc/mpc.h test_framework.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRCS) -o $(BIN)

clean:
	rm -rf build
