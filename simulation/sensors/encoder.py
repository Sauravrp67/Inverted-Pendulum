import numpy as np

# Only the pole angle wraps. The arm encoder counts full turns.


def measure(angles, noise, counts):
    steps = 2 * np.pi / np.asarray(counts)
    y = np.rint((np.asarray(angles) + noise) / steps) * steps
    y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))
    return y
