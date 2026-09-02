#include "pch.h"
#include "../Physics/DFSPHSolver.h"
#include "../Physics/DFSPHFluid.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/RigidFluidSolver.h"
#include "../Physics/ICollisionShape.h"
#include <cmath>
#include <memory>

using namespace Phantom::Physics;
using namespace Phantom::Math;

// Headless equivalent of PhysicsView's scenarios/51_couple_rigid_fluid_twoway.json:
// a DFSPH pool floating at y in [1.6, 2.2] and a 0.5-radius sphere released from
// y = 3 with Two-Way (Akinci boundary-particle) rigid-fluid coupling enabled.
//
// Mirrors FluidWorld::createDFSPH()/refreshCoupling()/stepOnce() (same rest-density
// calibration, same pressureCoe derivation, same per-frame syncBoundaries ->
// simulate -> stepForced order), so a regression here shows up in that scenario
// too, but runs in a second instead of needing the Vulkan app.
//
// This is the regression guard for internal design notes 1.6: before the
// fix the pool went NaN within a single frame and the sphere was flung around at
// 10^6 m/s.
namespace {

const float kRadius   = 0.3f;
const float kFrameDt  = 0.01f;
const float kGravityY = -9.8f;
const int   kFrames   = 40;

struct TwoWayDropScene {
    float effectLength = 0.6f;   // exactly what scenarios/51 asks for

    DFSPHFluid         fluid;
    DFSPHSolver        solver;
    RigidFluidSolver   world;
    SphereShape        sphere;
    RigidBody          body;
    RigidFluidBinding* binding = nullptr;

    void build(bool coupled) {
        fluid.setEffectLength(effectLength);
        fluid.viscosityCoe = 5.0f;
        // Same derivation as FluidWorld::createDFSPH(): pressureCoeScale=0.0833333
        // reproduces the same pressureCoe=0.05 the old formula gave for
        // SetFluidMaxDensityErrorRatio:235.2 at g 9.8.
        fluid.pressureCoe = WCSPHFluid::estimatePressureCoe(effectLength, 0.0833333f);

        const float diameter = kRadius * 2.0f;
        const float mass     = diameter * diameter * diameter;
        fluid.density = DFSPHSolver::calculateRestDensity(effectLength, kRadius, mass, &fluid);

        // Shallow pool: y in [1.6, 2.2], x/z in [-1.2, 1.2], spacing = diameter.
        for (float z = -1.2f; z <= 1.2f + 1e-4f; z += diameter)
            for (float y = 1.6f; y <= 2.2f + 1e-4f; y += diameter)
                for (float x = -1.2f; x <= 1.2f + 1e-4f; x += diameter)
                    fluid.createParticle(Vector3df(x, y, z), kRadius, mass);

        solver.setTimeStep(kFrameDt);
        solver.setBoundary(Box3df(Vector3df(-1000.f, 0.f, -1000.f),
                                  Vector3df(1000.f, 1000.f, 1000.f)), kFrameDt);
        solver.setExternalForce(Vector3df(0.f, kGravityY, 0.f));
        solver.add(&fluid);

        sphere.radius = 0.5f;
        body.position = { 0.f, 3.f, 0.f };
        body.setShape(&sphere);
        body.setMass(1.0f);
        world.rigidWorld().addBody(&body);
        world.rigidWorld().params().gravity = { 0.f, kGravityY, 0.f };
        world.rigidWorld().saveSnapshot();
        world.rigidWorld().setRunning(true);

        if (!coupled) return;

        binding = &world.bind(&body, &sphere, CouplingMode::TwoWay);
        binding->particles.sample(sphere, 0.2f);  // FluidWorld::couplingSpacing_
        binding->particles.computePsi(*fluid.getKernel(), fluid.getDensity());
        solver.addRigidBoundaryParticles(&binding->particles);
    }

    // One FluidWorld::stepOnce() frame.
    void step() {
        world.syncBoundaries();
        solver.simulate(kFrameDt, 2);
        world.stepForced(kFrameDt);
    }

    void run(int frames) { for (int i = 0; i < frames; ++i) step(); }

    float maxParticleY() const {
        float m = -1.0e30f;
        for (const auto& p : fluid.getParticles().positions) m = std::max(m, p.y);
        return m;
    }
    bool allParticlesFinite() const {
        for (const auto& p : fluid.getParticles().positions)
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
        return true;
    }
};

} // namespace

// Uncoupled control: the pool alone is a clean free-fall at this scale, so any
// divergence in the coupled tests below is the coupling's doing and not the
// pool's own configuration.
TEST(RigidFluidTwoWayDFSPHTest, UncoupledPoolFreeFalls) {
    TwoWayDropScene scene;
    scene.build(/*coupled=*/false);
    scene.run(kFrames);

    ASSERT_TRUE(scene.allParticlesFinite());
    // Semi-implicit Euler free fall over n steps: dy = -g dt^2 n(n+1)/2.
    const float drop = -kGravityY * kFrameDt * kFrameDt * kFrames * (kFrames + 1) * 0.5f;
    EXPECT_NEAR(scene.maxParticleY(), 2.2f - drop, 0.02f);
}

// The coupled pool must stay bounded. Until internal design notes 1.6
// was fixed, the boundary-particle density excess drove DFSPH's density-error
// solve to NaN inside the very first frame.
TEST(RigidFluidTwoWayDFSPHTest, CoupledPoolStaysFiniteAndBounded) {
    TwoWayDropScene scene;
    scene.build(/*coupled=*/true);
    scene.run(kFrames);

    ASSERT_TRUE(scene.allParticlesFinite());
    EXPECT_LT(scene.maxParticleY(), 2.4f);
    EXPECT_GT(scene.maxParticleY(), -0.5f);
}

// A sphere entering the pool must be *slowed* by the reaction, never sped up:
// compared against an identical uncoupled body under the same integrator, it
// has to end the run higher and slower.
TEST(RigidFluidTwoWayDFSPHTest, CoupledSphereIsSlowedRelativeToUncoupledControl) {
    TwoWayDropScene coupled;
    coupled.build(/*coupled=*/true);
    coupled.run(kFrames);

    TwoWayDropScene control;
    control.build(/*coupled=*/false);
    control.run(kFrames);

    ASSERT_TRUE(std::isfinite(coupled.body.linearVelocity.y));
    EXPECT_GT(coupled.body.linearVelocity.y, control.body.linearVelocity.y);
    EXPECT_GT(coupled.body.position.y,       control.body.position.y);
}

// The reaction the boundary particles hand back must be a net *upward* support
// once the sphere reaches the pool -- the original 1.6 report read the symptom
// as the coupling force adding to gravity, so pin the sign down directly.
TEST(RigidFluidTwoWayDFSPHTest, BoundaryReactionOnSphereIsUpward) {
    TwoWayDropScene scene;
    scene.build(/*coupled=*/true);

    float minReactionY = 1.0e30f;
    for (int i = 0; i < kFrames; ++i) {
        scene.step();
        Vector3df reaction(0.f, 0.f, 0.f);
        for (const auto& bp : scene.binding->particles.particles()) reaction += bp.accumForce;
        minReactionY = std::min(minReactionY, reaction.y);
    }

    EXPECT_GE(minReactionY, 0.f);
}

// RigidFluidSolver/SoftFluidSolver consume accumForce once per frame and treat
// it as a force acting over that whole frame, but simulate() adds to it once per
// CFL substep -- and the substep count is data dependent (it ran away to 10001
// on the diverging scene 1.6 was filed for). Splitting one frame into N substeps
// must therefore leave the accumulated reaction unchanged, not multiply it by N.
TEST(RigidFluidTwoWayDFSPHTest, ReactionIsFrameAveragedNotSubstepSummed) {
    TwoWayDropScene scene;
    scene.build(/*coupled=*/true);
    // Bring the sphere down to the pool so there is a reaction to measure.
    scene.run(35);

    // Rebuild the solver's working set the way simulate() does, then drive the
    // boundary coupling directly so the substep split is ours to choose.
    auto& soa = scene.fluid.getParticles();
    std::vector<DFSPHParticle> particles;
    for (size_t i = 0; i < soa.size(); ++i) particles.emplace_back(soa, i, &scene.fluid);

    std::vector<IBoundaryParticles*> boundaries{ &scene.binding->particles };

    // Once only: addBoundaryParticleDensity() accumulates, and both measurements
    // below must see the same density (and therefore the same per-substep force).
    scene.solver.addBoundaryParticleDensity(particles, boundaries);

    auto reactionOverOneFrame = [&](int substeps) {
        scene.binding->particles.clearAccumForce();
        for (int s = 0; s < substeps; ++s) {
            scene.solver.addBoundaryParticlePressure(particles, boundaries, kFrameDt / substeps);
        }
        Vector3df reaction(0.f, 0.f, 0.f);
        for (const auto& bp : scene.binding->particles.particles()) reaction += bp.accumForce;
        return reaction.y;
    };

    const float oneSubstep   = reactionOverOneFrame(1);
    const float eightSubsteps = reactionOverOneFrame(8);

    ASSERT_GT(oneSubstep, 0.f);
    EXPECT_NEAR(eightSubsteps, oneSubstep, std::abs(oneSubstep) * 0.01f);
}
