#include "controllers/classical/step_rate.h"
#include "controllers/mpc/mpc.h"
#include "controllers/state_feedback/lqr.h"
#include "controllers/swing_up/energy.h"
#include "controllers/classical/PIDController.h"

#include "estimators/kalman.h"

#include <array>
#include <cmath>
#include <vector>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using State = std::array<double, 4>;
using Matrix = py::array_t<double, py::array::c_style | py::array::forcecast>;
using FloatMatrix =
    py::array_t<float, py::array::c_style | py::array::forcecast>;

// Own the tables while Python uses the controller. The board keeps them in flash.
struct NativeMpc {
  std::vector<float> free_plan, inverse;
  LinearMpc solver;

  NativeMpc(const float *free, const float *inv, int n, float limit)
      : free_plan(free, free + n * 5), inverse(inv, inv + n * (n + 1) / 2),
        solver(free_plan.data(), inverse.data(), n, limit) {}
};

const double *matrix(const Matrix &a, int rows, int cols) {
  if (a.ndim() != 2 || a.shape(0) != rows || a.shape(1) != cols) {
    throw py::value_error("unexpected state-space matrix dimensions");
  }
  return a.data();
}

PYBIND11_MODULE(furuta_control, m) {
  m.doc() = "Native sampled controllers for the rotary pendulum";

  py::class_<PIDController>(m, "PID")
      .def(py::init(
          [](float p, float i, float d, float tau, float limit, bool aw) {
            PIDController pid(p, i, d);
            pid.setDerivativeTime(tau);
            pid.setOutputLimit(limit);
            pid.setConditionalIntegration(aw);
            return pid;
          }))
      .def("prime", &PIDController::prime)
      .def("step", [](PIDController &pid, double angle, double dt) {
        pid.update(-angle, dt);
        return py::make_tuple(pid.requested(), pid.integral());
      });

  py::class_<RateLead>(m, "RateLead")
      .def(py::init<>())
      .def("prime", &RateLead::prime)
      .def("step", &RateLead::step);

  py::class_<PulseTrim>(m, "PulseTrim")
      .def(py::init<>())
      .def("step", &PulseTrim::step);

  // Arm-position and rate feedback shifts the classical loop's tilt reference.
  m.def("arm_feedback",
        [](double arm_minus_target, double rate, double kp, double kd) {
          return kp * arm_minus_target + kd * rate;
        });

  m.def("lqr", [](const Matrix &k, const std::array<double, 5> &x, double ref) {
    return feedback(matrix(k, 1, 5), x.data(), ref, 5);
  });

  m.def("kalman", [](State x, const std::array<double, 2> &y, double u,
                     const Matrix &a, const Matrix &b, const Matrix &l) {
    observe(x.data(), y.data(), u, matrix(a, 4, 4), matrix(b, 4, 1),
            matrix(l, 4, 2));
    return x;
  });

  py::class_<NativeMpc>(m, "LinearMpc")
      .def(py::init(
          [](const FloatMatrix &free, const FloatMatrix &inverse, float limit) {
            if (free.ndim() != 2 || free.shape(1) != 5 || free.shape(0) < 1 ||
                free.shape(0) > LinearMpc::max_horizon) {
              throw py::value_error(
                  "MPC free-plan table must have 1..100 rows and 5 columns");
            }
            const int n = free.shape(0);
            if (inverse.ndim() != 1 || inverse.size() != n * (n + 1) / 2 ||
                !std::isfinite(limit) || limit <= 0) {
              throw py::value_error("invalid MPC inverse table or input limit");
            }
            for (const auto *table : {&free, &inverse}) {
              for (py::ssize_t i = 0; i < table->size(); ++i) {
                if (!std::isfinite(table->data()[i])) {
                  throw py::value_error("nonfinite MPC coefficient");
                }
              }
            }
            for (int i = 0; i < n; ++i) {
              if (inverse.data()[i * (i + 1) / 2 + i] <= 0) {
                throw py::value_error("MPC inverse diagonal must be positive");
              }
            }
            return std::make_unique<NativeMpc>(free.data(), inverse.data(), n,
                                               limit);
          }))
      .def("step",
           [](NativeMpc &m, const std::array<float, 5> &x, float ref) {
             float rate;
             bool solved = m.solver.step(x.data(), ref, rate);
             return py::make_tuple(rate, solved);
           })
      .def_property_readonly("plan",
                             [](const NativeMpc &m) {
                               return std::vector<float>(
                                   m.solver.plan(),
                                   m.solver.plan() + m.free_plan.size() / 5);
                             })
      .def_property_readonly(
          "sweeps", [](const NativeMpc &m) { return m.solver.sweeps(); });

  m.attr("MPC_MAX_SWEEPS") = LinearMpc::max_sweeps;
  m.attr("MPC_TOLERANCE") = LinearMpc::tolerance;

  m.def("pump_rate", &pump_rate);
}
