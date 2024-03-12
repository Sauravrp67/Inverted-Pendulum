"""Two revolute joints, one actuator. Lengths in m, masses in kg, torques in Nm."""

import numpy as np
from pydrake.all import (
    ApplySimulatorConfig,
    FixedOffsetFrame,
    Linearize,
    MultibodyPlant,
    RevoluteJoint,
    RigidTransform,
    Simulator,
    SimulatorConfig,
    SpatialInertia,
    UnitInertia,
)


def populate(plant, p, visuals=False):
    r, length, mass = p["arm_length"], p["rod_length"], p["rod_mass"]

    def link(name, m, size, axis):
        com = np.array(axis) * size / 2
        inertia = UnitInertia.ThinRod(size, axis).ShiftFromCenterOfMass(com)
        return plant.AddRigidBody(name, SpatialInertia(m, com, inertia))

    arm = link("arm", 3 * p["arm_inertia"] / r**2, r, [1, 0, 0])
    rod = link("pendulum", mass, length, [0, 0, 1])
    plant.mutable_gravity_field().set_gravity_vector([0, 0, -p["gravity"]])

    pivot = plant.AddFrame(
        FixedOffsetFrame("hinge", arm.body_frame(), RigidTransform([r, 0, 0]))
    )
    yaw = plant.AddJoint(
        RevoluteJoint(
            "azimuth",
            plant.world_frame(),
            arm.body_frame(),
            [0, 0, 1],
            p["arm_damping"],
        )
    )
    plant.AddJoint(
        RevoluteJoint("tilt", pivot, rod.body_frame(), [-1, 0, 0], p["hinge_damping"])
    )
    plant.AddJointActuator("motor", yaw)

    if visuals:
        from .geometry import register_visuals

        register_visuals(plant, arm, rod, r, length)
    plant.Finalize()


class DrakePlant:
    def __init__(self, p):
        self.system = MultibodyPlant(0.0)
        populate(self.system, p)
        self.j = p["rod_mass"] * p["rod_length"] ** 2 / 3
        self.g = p["rod_mass"] * p["gravity"] * p["rod_length"] / 2

    def linear(self):
        plant = self.system
        ctx = plant.CreateDefaultContext()
        port = plant.get_actuation_input_port()
        port.FixValue(ctx, [0.0])

        model = Linearize(
            plant, ctx, port.get_index(), plant.get_state_output_port().get_index()
        )
        return model.A(), model.B()

    def start(self, state, step):
        self.sim = Simulator(self.system)

        opts = SimulatorConfig()
        opts.integration_scheme = "runge_kutta5"
        opts.max_step_size = step
        opts.use_error_control = False
        ApplySimulatorConfig(opts, self.sim)

        self.ctx = self.sim.get_mutable_context()
        self.system.SetPositionsAndVelocities(self.ctx, state)
        self.system.get_actuation_input_port().FixValue(self.ctx, [0.0])
        self.sim.Initialize()

    def advance(self, time, command, pulse):
        self.system.get_actuation_input_port().FixValue(self.ctx, [command])
        self.system.get_applied_generalized_force_input_port().FixValue(
            self.ctx, [0.0, pulse]
        )

        self.sim.AdvanceTo(time)
        return self.system.GetPositionsAndVelocities(self.ctx).copy()
