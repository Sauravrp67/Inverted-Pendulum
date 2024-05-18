#include "test_framework.h"
#include "../controllers/mpc/mpc.h"

#include <limits>

namespace {
int calls, expire_after;

bool expired() { return ++calls > expire_after; }

float read_coefficient(const float *p) {
  ++calls;
  return *p;
}
} // namespace

TEST(mpc_coupled_input_bounds) {
  const float free[] = {2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const float inverse[] = {1, 0.5f, 1};
  float x[] = {1, 0, 0, 0, 0};
  float rate;
  calls = 0;
  LinearMpc mpc(free, inverse, 2, 1, read_coefficient);

  CHECK(mpc.step(x, 0, rate));
  CHECK_NEAR(rate, 1, 1e-6);
  CHECK_NEAR(mpc.plan()[1], -0.5, 1e-6);
  CHECK(calls > 0);

  x[0] = -1;
  CHECK(mpc.step(x, 0, rate));
  CHECK_NEAR(rate, -1, 1e-6);
  CHECK_NEAR(mpc.plan()[1], 0.5, 1e-6);
}

TEST(mpc_deadline_and_iteration_limit) {
  const float free[] = {2, 0, 0, 0, 0, -2, 0, 0, 0, 0};
  const float inverse[] = {1, 0.99f, 1};
  float x[] = {1, 0, 0, 0, 0};
  float rate = 7;
  LinearMpc mpc(free, inverse, 2, 1);

  for (int deadline : {0, 2, 3}) {
    calls = 0;
    expire_after = deadline;
    CHECK(!mpc.step(x, 0, rate, expired));
    CHECK_NEAR(rate, 0, 0);
  }

  CHECK(!mpc.step(x, 0, rate));
  CHECK(mpc.sweeps() == LinearMpc::max_sweeps);
  CHECK_NEAR(rate, 0, 0);

  x[0] = 0;
  CHECK(mpc.step(x, 0, rate));
  CHECK_NEAR(rate, 0, 0);
  CHECK(mpc.sweeps() == 1);
}

TEST(mpc_invalid_input) {
  const float free[] = {1, 0, 0, 0, 0};
  const float inverse[] = {1};
  float x[] = {0, 0, 0, 0, 0};
  float rate = 7;
  for (int n : {0, 101}) {
    LinearMpc mpc(free, inverse, n, 1);
    CHECK(!mpc.step(x, 0, rate));
    CHECK_NEAR(rate, 0, 0);
  }

  LinearMpc mpc(free, inverse, 1, 1);
  x[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(!mpc.step(x, 0, rate));
  CHECK_NEAR(rate, 0, 0);
}
