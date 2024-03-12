# Model conventions

Let `a` be the arm azimuth and `q` the rod angle from upright. Positive rod deflection is along the arm's positive tangential direction. The rod COM is at `r e_radial + l sin(q) e_tangent + l cos(q) e_vertical`, with arm radius `r` and rod half-length `l`. The renderer uses the full length for the tip.

For rod mass `m`, define `J = 4 m l² / 3` (inertia about the hinge), `b = m r l`, and `A = J_arm + m r²`. The rod is slender, with negligible inertia about its own long axis. Its kinetic and potential energies are

```text
2T = (A + J sin²q) a_dot² + 2 b cos(q) a_dot q_dot + J q_dot²
 V = m g l cos(q)
```

Drake constructs the equations from two rigid bodies with slender-rod spatial inertias. The arm joint is about world +z; the rod joint is about arm −x at the tip. The visible motor, bearing, floor and end cap are illustration geometry. For the independent analytical check, the Euler–Lagrange equations give

```text
M11 = A + J sin²q       M12 = M21 = b cos(q)       M22 = J
M [a_ddot, q_ddot]' =
  [ torque - d_arm a_dot - 2 J sin(q) cos(q) a_dot q_dot + b sin(q) q_dot²,
    pulse  - d_rod q_dot + m g l sin(q) + J sin(q) cos(q) a_dot² ]'
```

The actuator supplies only arm torque. A test disturbance enters only the second generalized force. Both hanging and upright rods are equilibria at zero velocity and zero input for any arm azimuth. At upright, evaluate the mass matrix at zero, keep the linear damping terms, and replace the gravity term with `m g l q`. This gives the continuous matrices returned by `Furuta.linear()`. The regulator and observer use the zero-order-hold discretisation of Drake's own upright linearisation. Tests compare the two continuous models and their unforced nonlinear trajectories.

The arm length (0.20 m), rod length (0.30 m), rod mass (0.10 kg), bare-arm inertia (0.005 kg m²), friction coefficients (Nm s/rad), torque limits (Nm) and simulated arm encoder resolution are illustrative design parameters. The pendulum encoder count follows the firmware. No system identification or motor torque calibration was performed.

In the showcase, the second force port carries an extra hinge loss, evaluated at the start of each 10 ms interval and held: `-dry*tanh(q_dot/smoothing) - drag*abs(q_dot)*q_dot`. This approximates sliding friction and rod drag, not static breakaway friction or a full aerodynamic model.

## From pulse rate to shaft torque

The 100 Hz controller issues a signed pulse rate. Every 1.25 ms the simulated driver accumulates rate times elapsed time, emits the integer part as signed steps, and retains the fractional remainder. The field angle `f` advances by `2*pi/steps_per_turn` per pulse. For that motor update the arm receives

```text
torque = clip(stiffness*(f - a) - damping*a_dot + torque_error, -limit, +limit)
```

Thus a stationary field can exert holding torque on a displaced shaft. `a` and `a_dot` in this equation belong to the physical plant, not to a hidden arm PID. The torque error is drawn once per controller tick, not once per integrator step. Pulses are batched, not edge-timed. This linear spring with damping and saturation approximates a motor following its field, not electromagnetic commutation, pole slipping or permanent missed steps. The 4,000 steps/s ceiling comes from the archived sketch; 1,600 steps/turn, 16 Nm/rad stiffness, 0.48 Nm s/rad damping and 0.65 Nm torque limit are not identified hardware parameters.

## Upright design

Append load angle `d = f - a` to the four mechanical states. Near upright and below saturation, the shaft torque is `stiffness*d - damping*a_dot`; the new state obeys `d_dot = u - a_dot`, where `u` is field speed in rad/s. The runner builds these five continuous equations from Drake's torque-input linearisation and discretises them at 10 ms. LQR and MPC use that five-state model, not the torque-input matrices with a rescaled gain.

Emitted pulse count is known without an encoder. The observer uses four relative coordinates `[a-f, q, a_dot, q_dot]`, measured `[a_encoder-f, q_encoder]` and the field speed inferred from emitted steps in the preceding interval. Only the estimator and state feedback reconstruct physical arm angle by adding the known field angle back. Neither motor noise nor truth velocities are fed to the controllers.
