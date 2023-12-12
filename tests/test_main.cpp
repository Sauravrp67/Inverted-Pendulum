// Entry point for the host test runner. Test cases self-register via the TEST
// macro in the other translation units; this just runs them all.
#include "test_framework.h"

int main() { return runAllTests(); }
