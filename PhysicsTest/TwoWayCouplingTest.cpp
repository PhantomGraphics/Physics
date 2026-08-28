#include "pch.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/RigidBoundary.h"
#include "../Physics/RigidBoundaryParticles.h"
#include "../Physics/RigidFluidSolver.h"
#include "../Physics/ICollisionShape.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);

// A thin horizontal "puddle" sitting only *below* y = -0.3, so any coupling
// reaction on a body pressing down into it is unambiguously upward -- unlike
// an all-around pool, which can null out a small/asymmetric reaction signal.
// Parameters (restDensity/stiffness/effectLength/spacing) match the stable
// regime confirmed by FluidPoolStabilityTest.PBSPH_PoolSettlesInBoxWithoutExploding.
void makePuddle(PBSPHFluid& fluid) {
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            Vector3df pos(-0.6f + 0.3f * i, -0.3f, -0.6f + 0.3f * j);
            fluid.createParticle(pos, 0.15f);
        }
    }
}

void configureFluid(PBSPHFluid& fluid) {
    fluid.setRestDensity(1.0f);
    fluid.setStiffness(0.001f);
    fluid.setVicsosity(0.0f);
    fluid.setEffectLength(1.0f);
    fluid.setIsBoundary(false);
}
}

// A sphere pressing down into a puddle from above should accumulate a net
// *upward* reaction on its boundary particles (buoyant support), and that
// reaction should measurably slow its downward acceleration compared to a
// free-falling control with no fluid coupling at all.
TEST(TwoWayCouplingTest, PBSPH_SpherePressingIntoPuddle_AccumulatesUpwardReaction) {
    PBSPHFluid fluid;
    configureFluid(fluid);
    makePuddle(fluid);

    PBSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce({ 0.f, 0.f, 0.f });  // puddle stays put; only the sphere has gravity
    solver.setTimeStep(0.005f);

    RigidFluidSolver world;
    SphereShape sphere;
    sphere.radius = 0.3f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, -0.15f, 0.f };  // overlaps the puddle from above
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.0f);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

    auto& binding = world.bind(body, &sphere, CouplingMode::TwoWay);
    binding.particles.sample(sphere, 0.15f);
    binding.particles.computePsi(*fluid.getKernel(), fluid.getRestDensity());
    solver.addRigidBoundaryParticles(&binding.particles);

    world.rigidWorld().params().gravity = { 0.f, -9.8f, 0.f };
    world.rigidWorld().saveSnapshot();
    world.rigidWorld().setRunning(true);

    const float dt = 0.005f;
    float accumForceYAtFirstStep = 0.f;
    for (int i = 0; i < 10; ++i) {
        world.syncBoundaries();
        solver.simulate(dt, 2);

        Vector3df accum(0.f, 0.f, 0.f);
        for (auto& bp : binding.particles.particles()) accum += bp.accumForce;
        if (i == 0) accumForceYAtFirstStep = accum.y;

        ASSERT_TRUE(std::isfinite(body->position.y));
        world.step(dt);
    }

    EXPECT_GT(accumForceYAtFirstStep, 0.f);

    // Free-fall control over the same duration, with no fluid coupling.
    const float freeFallVy = -9.8f * dt * 10.f;
    EXPECT_GT(body->linearVelocity.y, freeFallVy);
}

// The same puddle should also decelerate a ball that is already moving
// downward faster than gravity alone would carry it -- i.e. its downward
// speed should grow *less* over a few steps than an identical free-falling
// ball with no fluid coupling.
TEST(TwoWayCouplingTest, PBSPH_BallEnteringPuddle_VelocityDampedRelativeToFreeFall) {
    PBSPHFluid fluid;
    configureFluid(fluid);
    makePuddle(fluid);

    PBSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce({ 0.f, 0.f, 0.f });
    solver.setTimeStep(0.005f);

    RigidFluidSolver world;
    SphereShape sphere;
    sphere.radius = 0.3f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, -0.15f, 0.f };
    bodyOwned->linearVelocity = { 0.f, -2.0f, 0.f };
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.0f);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

    auto& binding = world.bind(body, &sphere, CouplingMode::TwoWay);
    binding.particles.sample(sphere, 0.15f);
    binding.particles.computePsi(*fluid.getKernel(), fluid.getRestDensity());
    solver.addRigidBoundaryParticles(&binding.particles);

    world.rigidWorld().params().gravity = { 0.f, -9.8f, 0.f };
    world.rigidWorld().saveSnapshot();
    world.rigidWorld().setRunning(true);

    const float dt = 0.005f;
    bool sawNonZeroReaction = false;
    for (int i = 0; i < 10; ++i) {
        world.syncBoundaries();
        solver.simulate(dt, 2);

        Vector3df accum(0.f, 0.f, 0.f);
        for (auto& bp : binding.particles.particles()) accum += bp.accumForce;
        if (getLength(accum) > 1.0e-6f) sawNonZeroReaction = true;

        ASSERT_TRUE(std::isfinite(body->linearVelocity.y));
        world.step(dt);
    }

    EXPECT_TRUE(sawNonZeroReaction);

    // Free-fall control from the same initial velocity, same duration.
    const float freeFallVy = -2.0f + (-9.8f * dt * 10.f);
    EXPECT_GT(body->linearVelocity.y, freeFallVy);
}

// Archimedes-like comparison: given the *same* buoyant support from the
// puddle (same shape, same starting depth), a lighter body should sink less
// than a heavier one, since buoyant force stays the same while weight does not.
TEST(TwoWayCouplingTest, PBSPH_LighterSphereSinksLessThanHeavierSphere) {
    auto runDrop = [](float mass) {
        PBSPHFluid fluid;
        configureFluid(fluid);
        makePuddle(fluid);

        PBSPHSolver solver;
        solver.add(&fluid);
        solver.setExternalForce({ 0.f, 0.f, 0.f });
        solver.setTimeStep(0.005f);

        RigidFluidSolver world;
        SphereShape sphere;
        sphere.radius = 0.3f;

        auto bodyOwned = std::make_unique<RigidBody>();
        bodyOwned->position = { 0.f, -0.15f, 0.f };
        bodyOwned->setShape(&sphere);
        bodyOwned->setMass(mass);
        RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

        auto& binding = world.bind(body, &sphere, CouplingMode::TwoWay);
        binding.particles.sample(sphere, 0.15f);
        binding.particles.computePsi(*fluid.getKernel(), fluid.getRestDensity());
        solver.addRigidBoundaryParticles(&binding.particles);

        world.rigidWorld().params().gravity = { 0.f, -9.8f, 0.f };
        world.rigidWorld().saveSnapshot();
        world.rigidWorld().setRunning(true);

        const float dt = 0.005f;
        for (int i = 0; i < 10; ++i) {
            world.syncBoundaries();
            solver.simulate(dt, 2);
            world.step(dt);
        }
        return body->position.y;
    };

    const float lightFinalY = runDrop(0.2f);
    const float heavyFinalY = runDrop(5.0f);

    EXPECT_GT(lightFinalY, heavyFinalY);
}
