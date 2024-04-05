"""Discrete stepper model, LQR gains and Kalman coefficients."""

import numpy as np
from scipy.linalg import cho_factor, cho_solve, solve_discrete_are
from scipy.signal import cont2discrete


def mpc_tables(A, B, c):
    # Stack x1 ... xN = O*x0 + T*[u0 ... uN-1].
    n = c["horizon"]
    O = np.vstack([np.linalg.matrix_power(A, i + 1) for i in range(n)])
    T = np.zeros((5 * n, n))

    for i in range(n):
        block = T[5 * i : 5 * i + 5]
        if i:
            block[:] = A @ T[5 * i - 5 : 5 * i]
        block[:, i : i + 1] = B

    weights = np.tile(c["state_weights"], n)
    H = (T.T * weights) @ T + c["rate_weight"] * np.eye(n)
    chol = cho_factor(H)
    free = -cho_solve(chol, (T.T * weights) @ O)
    inverse = cho_solve(chol, np.eye(n))

    return free.astype("float32"), inverse[np.tril_indices(n)].astype("float32")


def observer_model(A, B, dt):
    # The step count gives the field angle. Estimate arm angle relative to it.
    F = A[:4, :4].copy()
    F[:, 0] -= A[:4, 4]

    G = B[:4].copy()
    G[0, 0] -= dt
    return np.ascontiguousarray(F), G


def design(plant, c, *, with_lqr=True):
    # x = [arm, tilt, arm rate, tilt rate, field - arm]; u = field rad/s.
    M, N = plant.linear()
    p = c["stepper"]
    Ac = np.zeros((5, 5))
    Ac[:4, :4] = M
    Ac[:4, 2] -= N[:, 0] * p["damping_Nm_s_rad"]
    Ac[:4, 4] = N[:, 0] * p["stiffness_Nm_rad"]
    Ac[4, 2] = -1
    Bc = np.eye(5)[:, -1:]
    A, B, *_ = cont2discrete((Ac, Bc, np.eye(5), np.zeros((5, 1))), c["sample_s"])

    K = None
    if with_lqr:
        R = np.array([[c["rate_weight"]]])
        # Without an arm target, leave arm angle out of the LQR cost and model.
        first = int(c["arm_target_rad"] is None)
        a, b = A[first:, first:], B[first:]
        P = solve_discrete_are(a, b, np.diag(c["state_weights"][first:]), R)
        K = np.zeros((1, 5))
        K[:, first:] = np.linalg.solve(R + b.T @ P @ b, b.T @ P @ a)

    V = np.diag(
        (2 * np.pi / np.array(c["encoder_counts"])) ** 2 / 12
        + c["hardware_model"]["encoder_sd_rad"] ** 2
    )
    C = np.eye(4)[:2]
    F, _ = observer_model(A, B, c["sample_s"])
    S = solve_discrete_are(F.T, C.T, np.diag(c["process_variances"]), V)
    L = np.linalg.solve(C @ S @ C.T + V, C @ S).T
    return np.ascontiguousarray(A), np.ascontiguousarray(B), K, np.ascontiguousarray(L)
