# Host-side unit tests -- compiled with the system g++, no hardware required.
# Invoked by the top-level `make test`.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O1 -g
LIB      := ../lib

INCLUDES := -I. -I$(LIB)/PIDController -I$(LIB)/QuadratureEncoder

# Test sources + the platform-independent library sources under test. The
# QuadratureDecoder is header-only; PIDController needs its .cpp.
SRCS := test_main.cpp test_pid.cpp test_encoder.cpp \
        $(LIB)/PIDController/PIDController.cpp

BIN := build/run_tests

.PHONY: run build clean
run: build
	@./$(BIN)

build: $(BIN)

$(BIN): $(SRCS) test_framework.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRCS) -o $(BIN)

clean:
	rm -rf build
