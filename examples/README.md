# Examples

Standalone sketches that exercise the shared libraries (`lib/`) in isolation —
useful for bringing up hardware before running the full balancer.

These mirror the original experiment sketches (now in `archive/`) but are
rewritten against the library API to prove the libraries are reusable.

| Example | Libraries | Purpose |
|---|---|---|
| `encoder_readout/` | QuadratureEncoder | Print encoder angle on change |
| `stepper_accel_test/` | StepperDriver | Sweep the stepper to validate wiring |
| `rotary_to_stepper/` | QuadratureEncoder + StepperDriver | Slave the motor to the encoder |

## Building an example

The Makefile builds `src/*.cpp`. To flash an example instead of the balancer,
point the build at the example directory via `SRC_DIR`:

```sh
make BOARD=uno SRC_DIR=examples/encoder_readout build flash
```

(or temporarily replace `src/main.cpp` with the example). All examples use
`BoardConfig.h`, so they pick up the same pin map per board.
