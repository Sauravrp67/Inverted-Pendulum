#include "parameters.h"
#include "design_values.inc"

// On the Mega a finite host double can overflow a 32-bit target double.
#pragma GCC diagnostic error "-Woverflow"

_Static_assert(sizeof(IMAGE_LABEL) <= sizeof(params.image),
               "image label too long");

const FirmwareParameters params = {
    .image = IMAGE_LABEL,
    PARAMETER_VALUES,

    .rad_per_step = 6.283185307179586 / STEPS_PER_TURN,

    // Uniform rod, with its centre of mass at half its length.
    .hinge_inertia = ROD_MASS * ROD_LENGTH * ROD_LENGTH / 3.0,
    .gravity_moment = ROD_MASS * GRAVITY * ROD_LENGTH / 2.0,
    .coupling = ROD_MASS * ARM_LENGTH * ROD_LENGTH / 2.0,
};

const ObserverMatrices observer = {
    .F = OBSERVER_F, .G = OBSERVER_G, .L = OBSERVER_L};

#ifdef LQR_GAIN
const double lqr_gain[5] = LQR_GAIN;
#endif

#ifdef MPC_HORIZON
const float mpc_free_plan[MPC_HORIZON * 5] FIRMWARE_FLASH = MPC_FREE_PLAN;

const float mpc_inverse[MPC_HORIZON * (MPC_HORIZON + 1) / 2] FIRMWARE_FLASH =
    MPC_INVERSE;
#endif
