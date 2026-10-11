#pragma once

namespace Phantom {

/**
 * @brief Which domains one FluidWorld update advances, and which pairs exchange forces.
 *
 * Pure decision logic (no Vulkan / ImGui / solver types) so every Play / Pause / forced-Step /
 * coupling / external-clock combination can be tabulated in PhysicsTest. FluidWorld::step() and
 * stepOnce() build a StepInputs from their own state and then execute the returned plan; nothing
 * else decides the order.
 *
 * Execution order of a plan (FluidWorld::executePlan):
 *   1. boundary sync  -- only for the coupled pairs (rigidCoupled / softCoupled)
 *   2. fluid
 *   3. rigid          -- coupled: reaction applied + rigid advanced with the fluid's dt
 *                        (RigidFluidSolver::step); otherwise RigidBodyWorld::step()/stepForced()
 *   4. soft           -- same, via SoftFluidSolver or SoftBodyWorld
 * A coupled pair always advances by the fluid's time step, so forces exchanged inside the frame
 * are integrated over the span they were accumulated on.
 *
 * Clocks. While a FlameWorld is bound to the rigid world (RigidBodyWorld::hasExternalClock()),
 * that world's time is owned by the flame: rigid-fluid coupling is refused (rigidCoupled=false),
 * RigidBodyWorld::step() does nothing, and a forced step (stepForced) advances the flame's shared
 * clock exactly once. The flame itself is stepped by FluidApp / FlameWorld::step(), never here.
 *
 * Soft-body ownership. FluidWorld advances the soft world whenever soft coupling is *enabled*
 * (softOwnedByFluid), even if the active solver cannot do Track B and the pair therefore runs
 * uncoupled; callers step the soft world themselves only when coupling is disabled. (stepOnce()
 * used to skip the soft world in the "enabled but unsupported" case, so neither side stepped it.)
 */
struct StepInputs {
    bool forced = false;             ///< stepOnce(): ignore the Play/Pause flags below.

    bool fluidRunning = false;       ///< Play/Pause of each domain (ignored when forced).
    bool rigidRunning = false;
    bool softRunning  = false;

    bool rigidCouplingEnabled = false;
    bool rigidHasExternalClock = false;  ///< a FlameWorld owns the rigid world's clock.
    bool oneWaySupported = false;        ///< active fluid has a CPU solver to bind to.

    bool softCouplingEnabled = false;
    bool softWorldPresent = false;
    bool twoWaySupported = false;        ///< active solver implements Track B boundary particles.
};

struct StepPlan {
    bool fluid = false;
    bool rigid = false;
    bool soft = false;
    bool rigidCoupled = false;
    bool softCoupled = false;
    bool softOwnedByFluid = false;   ///< the fluid world steps the soft world; the caller must not.
};

inline StepPlan planStep(const StepInputs& in)
{
    StepPlan p;
    const bool softBound = in.softCouplingEnabled && in.softWorldPresent;
    p.softOwnedByFluid = softBound;

    if (in.forced) {
        p.fluid = true;
        p.rigid = true;                  // under an external clock this ticks the flame once
        p.soft  = softBound;
        p.rigidCoupled = !in.rigidHasExternalClock && in.rigidCouplingEnabled && in.oneWaySupported;
        p.softCoupled  = softBound && in.twoWaySupported;
        return p;
    }

    p.fluid = in.fluidRunning;
    p.rigid = in.rigidRunning;
    p.soft  = softBound && in.softRunning;
    p.rigidCoupled = !in.rigidHasExternalClock && in.rigidCouplingEnabled &&
                     p.fluid && p.rigid && in.oneWaySupported;
    p.softCoupled  = p.soft && p.fluid && in.twoWaySupported;
    return p;
}

} // namespace Phantom
