// Minimal zero-dependency unit-test harness for the host build.
//
// Register cases with TEST(name) { ... } and assert with CHECK / CHECK_NEAR.
// A single test_main.cpp links all cases and runs them; any failure makes the
// process exit non-zero so `make test` fails the build.
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <cmath>
#include <cstdio>
#include <vector>

struct TestCase {
  const char* name;
  void (*fn)();
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> cases;
  return cases;
}

inline int& failureCount() {
  static int failures = 0;
  return failures;
}

struct TestRegistrar {
  TestRegistrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

#define TEST(name)                                                       \
  static void name();                                                    \
  static TestRegistrar registrar_##name(#name, name);                    \
  static void name()

#define CHECK(cond)                                                      \
  do {                                                                   \
    if (!(cond)) {                                                       \
      std::printf("    FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__, #cond); \
      ++failureCount();                                                  \
    }                                                                    \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                            \
  do {                                                                   \
    double da = (a), db = (b);                                          \
    if (std::fabs(da - db) > (eps)) {                                    \
      std::printf("    FAIL %s:%d  CHECK_NEAR(%s=%g, %s=%g, %g)\n",       \
                  __FILE__, __LINE__, #a, da, #b, db, (double)(eps));     \
      ++failureCount();                                                  \
    }                                                                    \
  } while (0)

inline int runAllTests() {
  std::printf("Running %zu test case(s)\n", registry().size());
  for (const auto& tc : registry()) {
    int before = failureCount();
    tc.fn();
    std::printf("  [%s] %s\n", failureCount() == before ? "PASS" : "FAIL", tc.name);
  }
  if (failureCount() == 0) {
    std::printf("All tests passed.\n");
    return 0;
  }
  std::printf("%d check(s) failed.\n", failureCount());
  return 1;
}

#endif  // TEST_FRAMEWORK_H
