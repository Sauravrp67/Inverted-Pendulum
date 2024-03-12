"""Visual-only bench, encoder markings, motor housing, arm and rod."""

import numpy as np
from pydrake.all import Box, Cylinder, RotationMatrix, RigidTransform, Sphere


def register_visuals(plant, arm, rod, r, length):
    world = plant.world_body()

    def draw(body, name, shape, xyz, rgb, rot=None):
        pose = RigidTransform(rot if rot is not None else RotationMatrix(), xyz)
        plant.RegisterVisualGeometry(body, pose, shape, name, [*rgb, 1])

    metal, ink, orange = [0.52, 0.63, 0.68], [0.11, 0.16, 0.19], [0.95, 0.33, 0.07]
    floor = -length - 0.055
    draw(
        world,
        "bench",
        Box(1.8, 1.8, 0.025),
        [0, 0, floor - 0.015],
        [0.82, 0.86, 0.87],
    )
    draw(world, "foot", Box(0.18, 0.16, 0.025), [0, 0, floor + 0.013], ink)
    draw(world, "column", Cylinder(0.032, -floor - 0.05), [0, 0, floor / 2], metal)
    draw(world, "motor_case", Cylinder(0.048, 0.052), [0, 0, -0.041], ink)
    for i in range(4):
        draw(
            world,
            f"motor_fin_{i}",
            Cylinder(0.05, 0.003),
            [0, 0, -0.025 - i * 0.009],
            metal,
        )
        draw(
            world,
            f"foot_bolt_{i}",
            Cylinder(0.006, 0.005),
            [
                0.071 * (1 if i % 2 else -1),
                0.061 * (1 if i // 2 else -1),
                floor + 0.027,
            ],
            metal,
        )

    draw(arm, "hub", Cylinder(0.036, 0.032), [0, 0, -0.008], orange)
    draw(arm, "beam", Box(r, 0.025, 0.018), [r / 2, 0, 0], [0.035, 0.41, 0.50])
    draw(
        arm,
        "bearing",
        Cylinder(0.021, 0.039),
        [r, 0, 0],
        ink,
        RotationMatrix.MakeYRotation(np.pi / 2),
    )
    draw(
        rod,
        "axle",
        Cylinder(0.012, 0.043),
        [0, 0, 0],
        metal,
        RotationMatrix.MakeYRotation(np.pi / 2),
    )
    draw(rod, "shaft", Cylinder(0.006, length), [0, 0, length / 2], orange)
    draw(rod, "end_cap", Sphere(0.011), [0, 0, length], ink)

    for i in range(24):
        theta = i * np.pi / 12
        draw(
            world,
            f"azimuth_{i}",
            Box(0.023 if i % 3 == 0 else 0.012, 0.003, 0.002),
            [(r + 0.05) * np.cos(theta), (r + 0.05) * np.sin(theta), floor],
            metal,
            RotationMatrix.MakeZRotation(theta),
        )
