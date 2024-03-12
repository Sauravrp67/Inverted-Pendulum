"""Rigid horizontal arm and slender hinged rod. SI throughout, upright = 0."""

from math import sin, cos
import numpy as np


class Furuta:
    def __init__(self, p):
        self.r, self.l = p["arm_length"], p["rod_length"] / 2
        m = p["rod_mass"]
        self.a = p["arm_inertia"] + m * self.r**2
        self.b = m * self.r * self.l
        self.j = 4 * m * self.l**2 / 3
        self.g = m * p["gravity"] * self.l
        self.da, self.dp = p["arm_damping"], p["hinge_damping"]

    def rhs(self, x, torque, pulse=0):
        _, angle, w, v = x
        s, c = sin(angle), cos(angle)

        a, b = self.a + self.j * s * s, self.b * c
        f = torque - self.da * w - 2 * self.j * s * c * v * w + self.b * s * v * v
        h = pulse - self.dp * v + self.g * s + self.j * s * c * w * w

        det = a * self.j - b * b
        arm_accel = (self.j * f - b * h) / det
        rod_accel = (a * h - b * f) / det
        return np.array([w, v, arm_accel, rod_accel])

    def energy(self, x):
        _, angle, w, v = x
        return (
            0.5 * (self.a + self.j * sin(angle) ** 2) * w * w
            + self.b * cos(angle) * w * v
            + 0.5 * self.j * v * v
            + self.g * cos(angle)
        )

    def linear(self):
        mass = np.array([[self.a, self.b], [self.b, self.j]])
        A = np.zeros((4, 4))
        A[:2, 2:] = np.eye(2)
        A[2:] = np.linalg.solve(mass, [[0, 0, -self.da, 0], [0, self.g, 0, -self.dp]])
        B = np.r_[0, 0, np.linalg.solve(mass, [1, 0])][:, None]
        return A, B


def rk4(rhs, x, h):
    a = rhs(x)
    b = rhs(x + h * a / 2)
    c = rhs(x + h * b / 2)
    return x + h * (a + 2 * b + 2 * c + rhs(x + h * c)) / 6
