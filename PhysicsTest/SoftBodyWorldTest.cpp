#include "pch.h"
#include "../Physics/SoftBodySolver.h"
#include "../Physics/ClothBody.h"

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {

float maxSpeed(const ClothBody& cloth) {
    float m = 0.f;
    for (const auto& v : cloth.getMesh().particles.velocities)
        m = std::max(m, glm::length(v));
    return m;
}

} // namespace

// NOTE: SoftBodySolver::ClothWithSphere_SettlesWithoutVelocityExplosion
// (the "no velocity explosion when cloth settles onto an overlapping sphere"
// regression, ~7.2s of step-loop) moved to
// Physics/PhysicsView/scenarios/cloth_sphere.json, which exercises the same
// SoftBodyWorld::SoftBodyPreset::ClothWithSphere preset via
// SetPreset:ClothWithSphere + Step:200 + GetMaxSpeed range check.

TEST(SoftBodyWorldTest, StepUnconditional_AdvancesEvenWhenNotRunning) {
    SoftBodySolver world;
    world.setRunning(false);
    world.solverParams().gravity = { 0.f, -9.8f, 0.f };

    ClothBodyParams p;
    p.rows = 3;
    p.cols = 3;
    p.width = 1.f;
    p.height = 1.f;
    p.pinTopLeft = false;
    p.pinTopRight = false;
    p.pinTopEdge = false;
    p.origin = { 0.f, 1.f, 0.f };  // clear of the y=0 floor collider so gravity isn't immediately cancelled
    auto clothOwned = std::make_unique<ClothBody>(p);
    auto* cloth = clothOwned.get();
    world.addBody(cloth);

    EXPECT_NEAR(maxSpeed(*cloth), 0.f, 1.0e-6f);

    world.stepUnconditional();

    EXPECT_GT(maxSpeed(*cloth), 0.f) << "stepUnconditional() must advance the simulation regardless of isRunning()";
}
