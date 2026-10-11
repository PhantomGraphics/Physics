#include "pch.h"
#include "../PhysicsView/StepPlan.h"

using namespace Phantom;

namespace {

StepInputs running(bool fluid, bool rigid, bool soft)
{
    StepInputs in;
    in.fluidRunning = fluid;
    in.rigidRunning = rigid;
    in.softRunning = soft;
    in.oneWaySupported = true;
    in.twoWaySupported = true;
    in.softWorldPresent = true;
    return in;
}

} // namespace

TEST(StepPlanTest, EverythingPausedStepsNothing)
{
    StepInputs in = running(false, false, false);
    in.rigidCouplingEnabled = true;
    in.softCouplingEnabled = true;
    const StepPlan p = planStep(in);
    EXPECT_FALSE(p.fluid);
    EXPECT_FALSE(p.rigid);
    EXPECT_FALSE(p.soft);
    EXPECT_FALSE(p.rigidCoupled);
    EXPECT_FALSE(p.softCoupled);
}

TEST(StepPlanTest, DomainsAdvanceIndependentlyWhenUncoupled)
{
    StepPlan p = planStep(running(true, false, false));
    EXPECT_TRUE(p.fluid);  EXPECT_FALSE(p.rigid);

    p = planStep(running(false, true, false));
    EXPECT_FALSE(p.fluid); EXPECT_TRUE(p.rigid);

    p = planStep(running(true, true, false));
    EXPECT_TRUE(p.fluid);  EXPECT_TRUE(p.rigid);
    EXPECT_FALSE(p.rigidCoupled);
}

TEST(StepPlanTest, RigidCouplingNeedsBothSidesRunning)
{
    StepInputs in = running(true, true, false);
    in.rigidCouplingEnabled = true;
    EXPECT_TRUE(planStep(in).rigidCoupled);

    in.rigidRunning = false;     // paused rigid world: fluid steps alone, nothing exchanged
    StepPlan p = planStep(in);
    EXPECT_TRUE(p.fluid);
    EXPECT_FALSE(p.rigid);
    EXPECT_FALSE(p.rigidCoupled);

    in.rigidRunning = true;
    in.fluidRunning = false;
    p = planStep(in);
    EXPECT_FALSE(p.fluid);
    EXPECT_TRUE(p.rigid);
    EXPECT_FALSE(p.rigidCoupled);
}

TEST(StepPlanTest, ExternalClockOwnsTheRigidWorldAndRefusesCoupling)
{
    StepInputs in = running(true, true, false);
    in.rigidCouplingEnabled = true;
    in.rigidHasExternalClock = true;
    StepPlan p = planStep(in);
    EXPECT_FALSE(p.rigidCoupled);
    EXPECT_TRUE(p.rigid);        // RigidBodyWorld::step() is a no-op under an external clock

    in.forced = true;            // a forced step ticks the shared clock once, still uncoupled
    p = planStep(in);
    EXPECT_FALSE(p.rigidCoupled);
    EXPECT_TRUE(p.rigid);
}

TEST(StepPlanTest, GpuFluidCannotBeCoupled)
{
    StepInputs in = running(true, true, true);
    in.rigidCouplingEnabled = true;
    in.softCouplingEnabled = true;
    in.oneWaySupported = false;
    in.twoWaySupported = false;
    for (bool forced : {false, true}) {
        in.forced = forced;
        const StepPlan p = planStep(in);
        EXPECT_FALSE(p.rigidCoupled);
        EXPECT_FALSE(p.softCoupled);
        EXPECT_TRUE(p.soft) << "an uncoupled soft world still advances";
    }
}

TEST(StepPlanTest, SoftWorldIsOwnedByTheFluidOnlyWhileCouplingIsEnabled)
{
    StepInputs in = running(true, false, true);
    EXPECT_FALSE(planStep(in).softOwnedByFluid);
    EXPECT_FALSE(planStep(in).soft);          // the caller steps it

    in.softCouplingEnabled = true;
    StepPlan p = planStep(in);
    EXPECT_TRUE(p.softOwnedByFluid);
    EXPECT_TRUE(p.soft);
    EXPECT_TRUE(p.softCoupled);

    in.softWorldPresent = false;
    p = planStep(in);
    EXPECT_FALSE(p.softOwnedByFluid);
    EXPECT_FALSE(p.soft);
}

TEST(StepPlanTest, PausedFluidLetsAnEnabledSoftWorldRunUncoupled)
{
    StepInputs in = running(false, false, true);
    in.softCouplingEnabled = true;
    const StepPlan p = planStep(in);
    EXPECT_FALSE(p.fluid);
    EXPECT_TRUE(p.soft);
    EXPECT_FALSE(p.softCoupled);
}

TEST(StepPlanTest, ForcedStepIgnoresPlayPauseFlags)
{
    StepInputs in = running(false, false, false);
    in.forced = true;
    StepPlan p = planStep(in);
    EXPECT_TRUE(p.fluid);
    EXPECT_TRUE(p.rigid);
    EXPECT_FALSE(p.soft);                // soft coupling disabled: caller's job

    in.rigidCouplingEnabled = true;
    in.softCouplingEnabled = true;
    p = planStep(in);
    EXPECT_TRUE(p.rigidCoupled);
    EXPECT_TRUE(p.softCoupled);
    EXPECT_TRUE(p.soft);
}

TEST(StepPlanTest, ForcedStepAdvancesSoftWorldWhenCouplingIsEnabledButUnsupported)
{
    // Regression: stepOnce() skipped the soft world in this case while the callers also skipped it
    // (they step only when coupling is disabled), so the soft world never advanced.
    StepInputs in = running(false, false, false);
    in.forced = true;
    in.softCouplingEnabled = true;
    in.twoWaySupported = false;          // e.g. no fluid built yet / GPU_CSPH
    const StepPlan p = planStep(in);
    EXPECT_TRUE(p.soft);
    EXPECT_FALSE(p.softCoupled);
    EXPECT_TRUE(p.softOwnedByFluid);
}

TEST(StepPlanTest, InvariantsHoldForEveryInputCombination)
{
    constexpr int kBits = 11;
    for (int mask = 0; mask < (1 << kBits); ++mask) {
        auto bit = [&](int i) { return (mask >> i & 1) != 0; };
        StepInputs in;
        in.forced = bit(0);
        in.fluidRunning = bit(1);
        in.rigidRunning = bit(2);
        in.softRunning = bit(3);
        in.rigidCouplingEnabled = bit(4);
        in.rigidHasExternalClock = bit(5);
        in.oneWaySupported = bit(6);
        in.softCouplingEnabled = bit(7);
        in.softWorldPresent = bit(8);
        in.twoWaySupported = bit(9);
        // bit 10: spare, doubles the sweep to confirm the plan has no hidden dependence on it.
        const StepPlan p = planStep(in);
        SCOPED_TRACE(mask);

        // A coupled pair exchanges forces with a fluid that advances in the same plan.
        if (p.rigidCoupled) { EXPECT_TRUE(p.fluid); EXPECT_TRUE(p.rigid); }
        if (p.softCoupled)  { EXPECT_TRUE(p.fluid); EXPECT_TRUE(p.soft); }
        // The flame's clock excludes rigid-fluid coupling; coupling needs its own prerequisites.
        if (p.rigidCoupled) { EXPECT_FALSE(in.rigidHasExternalClock); EXPECT_TRUE(in.rigidCouplingEnabled); EXPECT_TRUE(in.oneWaySupported); }
        if (p.softCoupled)  { EXPECT_TRUE(in.softCouplingEnabled); EXPECT_TRUE(in.softWorldPresent); EXPECT_TRUE(in.twoWaySupported); }
        // The fluid world steps the soft world iff coupling is enabled; never otherwise.
        EXPECT_EQ(p.softOwnedByFluid, in.softCouplingEnabled && in.softWorldPresent);
        if (p.soft) EXPECT_TRUE(p.softOwnedByFluid);

        if (in.forced) {
            EXPECT_TRUE(p.fluid);
            EXPECT_TRUE(p.rigid);
            EXPECT_EQ(p.soft, p.softOwnedByFluid);
        } else {
            EXPECT_EQ(p.fluid, in.fluidRunning);
            EXPECT_EQ(p.rigid, in.rigidRunning);
            EXPECT_EQ(p.soft, p.softOwnedByFluid && in.softRunning);
        }
    }
}
