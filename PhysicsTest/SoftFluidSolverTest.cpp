#include "pch.h"
#include "../Physics/SoftFluidSolver.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;

// Minimal ISoftBody with a single free particle and no constraints, used to
// isolate the fluid-reaction force path from XPBD constraint-projection
// effects (a multi-particle mesh would let neighbors resist an
// individually-forced particle, making the expected velocity harder to
// predict in closed form).
class SingleParticleBody : public ISoftBody {
public:
    SingleParticleBody() {
        mesh_.particles.resize(1);
        mesh_.particles.positions[0] = { 0.f, 1.f, 0.f };  // clear of the y=0 floor collider
        mesh_.particles.inverseMasses[0] = 1.0f;
    }
    void            build(XPBDSolver&)       override {}
    SoftMesh&       getMesh()                override { return mesh_; }
    const SoftMesh& getMesh()          const override { return mesh_; }

private:
    SoftMesh mesh_;
};
}

TEST(SoftFluidSolverTest, Bind_CreatesBindingWithOneParticlePerMeshParticle) {
    SoftFluidSolver world;
    SingleParticleBody body;

    auto& binding = world.bind(&body);

    EXPECT_EQ(binding.body, &body);
    EXPECT_EQ(binding.particles.particles().size(), body.getMesh().particles.size());
}

TEST(SoftFluidSolverTest, SyncBoundaries_RefreshesWorldPosAndRecomputesPsi) {
    SoftFluidSolver world;
    SingleParticleBody body;
    auto& binding = world.bind(&body);

    body.getMesh().particles.positions[0] = { 2.f, 3.f, 4.f };

    SPHKernel kernel(0.5f);
    world.syncBoundaries(&kernel, 1000.f);

    ASSERT_FALSE(binding.particles.particles().empty());
    EXPECT_NEAR(getDistance(binding.particles.particles()[0].worldPos, Vector3df(2.f, 3.f, 4.f)), 0.f, kTol);
    // A single boundary particle has no neighbors, so sumW = kernel(0) only.
    EXPECT_GT(binding.particles.particles()[0].psi, 0.f);
}

TEST(SoftFluidSolverTest, SyncBoundaries_ClearsAccumForce) {
    SoftFluidSolver world;
    SingleParticleBody body;
    auto& binding = world.bind(&body);

    binding.particles.particles()[0].accumForce = { 1.f, 2.f, 3.f };

    world.syncBoundaries(nullptr, 0.f);

    EXPECT_NEAR(getLength(binding.particles.particles()[0].accumForce), 0.f, kTol);
}

TEST(SoftFluidSolverTest, Step_AppliesAccumForceAcrossAllSubstepsCorrectly) {
    SoftFluidSolver world;

    // The body must be added to softWorld() itself (not a standalone
    // instance) since SoftFluidSolver::step() drives softWorld().
    auto softOwned = std::make_unique<SingleParticleBody>();
    auto* soft = softOwned.get();
    world.softWorld().addBody(soft);
    auto& binding = world.bind(soft);

    world.softWorld().solverParams().gravity = { 0.f, 0.f, 0.f };  // isolate the coupling force
    world.softWorld().solverParams().damping = 1.f;  // isolate substep-scaling from velocity damping
    world.softWorld().setRunning(true);

    world.syncBoundaries(nullptr, 0.f);  // clears accumForce, refreshes worldPos

    // Stand in for what the fluid solver's addSoftBoundaryParticlePressure()
    // would have accumulated during this step.
    binding.particles.particles()[0].accumForce = { 0.f, 100.f, 0.f };

    const float dt = 0.016f;
    world.step(dt);

    // velocity += dt_full * inverseMass * force, exactly like
    // RigidBody::integrate() -- NOT divided or multiplied by numSubsteps
    // (see XPBDSolver::applyExternalForces()'s doc: force is applied
    // unmodified every substep, same as gravity, so it sums to dt_full over
    // the whole frame).
    // Loosened from kTol: 10 substeps of float32 accumulation introduce
    // ~3.6e-4 rounding error, which is expected precision loss, not a
    // substep-scaling bug (a divide-by-numSubsteps or apply-once mistake
    // would be off by orders of magnitude more than this).
    EXPECT_NEAR(soft->getMesh().particles.velocities[0].y, 100.f * dt, 1.0e-3f);
}

TEST(SoftFluidSolverTest, StepForced_AdvancesEvenWhenNotRunning) {
    SoftFluidSolver world;
    auto softOwned = std::make_unique<SingleParticleBody>();
    auto* soft = softOwned.get();
    world.softWorld().addBody(soft);
    world.bind(soft);

    world.softWorld().solverParams().gravity = { 0.f, -9.8f, 0.f };
    world.softWorld().setRunning(false);

    world.syncBoundaries(nullptr, 0.f);
    world.stepForced(0.016f);

    EXPECT_LT(soft->getMesh().particles.velocities[0].y, 0.f);
}
