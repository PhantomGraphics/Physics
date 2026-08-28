#include "pch.h"
#include "../Physics/RigidFluidSolver.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;
}

TEST(RigidFluidSolverTest, Bind_CreatesBoundaryWithBodyPose) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 1.f, 2.f, 3.f };
    bodyOwned->setShape(&sphere);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

    auto& binding = world.bind(body, &sphere, CouplingMode::OneWay);

    EXPECT_EQ(binding.body, body);
    EXPECT_EQ(binding.mode, CouplingMode::OneWay);
    EXPECT_NEAR(getDistance(binding.boundary.getPosition(), body->position), 0.f, kTol);
}

TEST(RigidFluidSolverTest, SyncBoundaries_ReflectsBodyMovementAfterStep) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, 5.f, 0.f };
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

    auto& binding = world.bind(body, &sphere, CouplingMode::OneWay);

    world.rigidWorld().params().gravity = { 0.f, -9.8f, 0.f };
    world.rigidWorld().saveSnapshot();
    world.rigidWorld().setRunning(true);

    world.syncBoundaries();
    world.step(0.016f);

    // body fell, but boundary was synced BEFORE the step -> still at the old pose.
    EXPECT_NEAR(binding.boundary.getPosition().y, 5.f, kTol);
    EXPECT_LT(body->position.y, 5.f);

    world.syncBoundaries();
    // now the boundary reflects the body's post-step pose.
    EXPECT_NEAR(binding.boundary.getPosition().y, body->position.y, kTol);
}

TEST(RigidFluidSolverTest, Step_IntegratesGravityLikeRigidBodyWorld) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);
    world.bind(body, &sphere, CouplingMode::OneWay);

    world.rigidWorld().params().gravity = { 0.f, -9.8f, 0.f };
    world.rigidWorld().saveSnapshot();
    world.rigidWorld().setRunning(true);

    world.step(0.016f);

    float expectedVy = -9.8f * 0.016f;
    EXPECT_NEAR(body->linearVelocity.y, expectedVy, kTol);
}

TEST(RigidFluidSolverTest, Step_NoOpWhenRigidWorldNotRunning) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, 5.f, 0.f };
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);
    world.bind(body, &sphere, CouplingMode::OneWay);

    world.rigidWorld().params().gravity = { 0.f, -9.8f, 0.f };
    // running_ defaults to false and is never enabled here.

    world.step(0.016f);

    EXPECT_NEAR(body->position.y, 5.f, kTol);
    EXPECT_NEAR(body->linearVelocity.y, 0.f, kTol);
}

TEST(RigidFluidSolverTest, OneWayBinding_BoundaryPushesFluidParticleOut) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, 0.f, 0.f };
    bodyOwned->setShape(&sphere);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

    auto& binding = world.bind(body, &sphere, CouplingMode::OneWay);
    binding.boundary.setPenaltyStiffness(1000.f);
    world.syncBoundaries();

    // A fluid particle penetrating the sphere by 0.2 should be pushed outward.
    Vector3df particlePos(0.3f, 0.f, 0.f);
    Vector3df force = binding.boundary.getBoundaryForce(particlePos);

    EXPECT_NEAR(force.x, 1000.f * 0.2f, kTol);
    EXPECT_NEAR(force.y, 0.f, kTol);
    EXPECT_NEAR(force.z, 0.f, kTol);
}

TEST(RigidFluidSolverTest, TwoWayBinding_AccumulatedReactionMovesBody) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

    auto& binding = world.bind(body, &sphere, CouplingMode::TwoWay);
    binding.particles.sample(sphere, 0.2f);
    ASSERT_FALSE(binding.particles.particles().empty());

    world.rigidWorld().params().gravity = { 0.f, 0.f, 0.f };  // isolate the coupling force
    world.rigidWorld().saveSnapshot();
    world.rigidWorld().setRunning(true);

    world.syncBoundaries();  // clears accumForce and refreshes worldPos

    // Stand in for what the fluid solver's addRigidBoundaryParticlePressure()
    // would have accumulated during this step.
    binding.particles.particles().front().accumForce = { 0.f, 100.f, 0.f };

    world.step(0.016f);

    // linearVelocity += dt * inverseMass * forceAccum, mass=1 -> dt*100.
    EXPECT_NEAR(body->linearVelocity.y, 100.f * 0.016f, kTol);
}

// 2.3-A (docs/todo/PLAN_physics_ownership_and_coupling_unification.md): callers
// register `&binding.boundary`/`&binding.particles` with the fluid solver(s)
// immediately after bind() returns. Because bindings_ used to be a
// std::vector<RigidFluidBinding> held by value, a later bind() could
// reallocate the backing storage and leave every previously-registered
// pointer dangling. This test binds 3 bodies (mirroring
// FluidWorld::refreshCoupling()'s "one bind() per body" loop over >=2 dynamic
// rigid bodies) and checks that a pointer captured right after the first
// bind() still points at the live binding after two more bind() calls.
// Expected RED until Phase 2 switches bindings_ to an address-stable
// container (std::deque).
TEST(RigidFluidSolverTest, Bind_KeepsEarlierBindingPointersStableAcrossLaterBinds) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    std::vector<std::unique_ptr<RigidBody>> bodies;
    for (int i = 0; i < 3; ++i) {
        auto body = std::make_unique<RigidBody>();
        body->position = { static_cast<float>(i), 0.f, 0.f };
        body->setShape(&sphere);
        world.rigidWorld().addBody(body.get());
        bodies.push_back(std::move(body));
    }

    auto& firstBinding = world.bind(bodies[0].get(), &sphere, CouplingMode::OneWay);
    RigidBoundary* registeredFirstBoundary = &firstBinding.boundary;

    world.bind(bodies[1].get(), &sphere, CouplingMode::OneWay);
    world.bind(bodies[2].get(), &sphere, CouplingMode::OneWay);

    ASSERT_EQ(world.getBindings().size(), 3U);
    EXPECT_EQ(registeredFirstBoundary, &world.getBindings()[0].boundary)
        << "pointer captured after the first bind() must stay valid once later "
           "bind() calls have been made (address-stable bindings_ container)";
}

TEST(RigidFluidSolverTest, SyncBoundaries_ClearsTwoWayAccumForce) {
    RigidFluidSolver world;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->setShape(&sphere);
    RigidBody* body = bodyOwned.get();
    world.rigidWorld().addBody(body);

    auto& binding = world.bind(body, &sphere, CouplingMode::TwoWay);
    binding.particles.sample(sphere, 0.2f);
    ASSERT_FALSE(binding.particles.particles().empty());

    for (auto& bp : binding.particles.particles()) bp.accumForce = { 1.f, 2.f, 3.f };

    world.syncBoundaries();

    for (auto& bp : binding.particles.particles()) {
        EXPECT_NEAR(getLength(bp.accumForce), 0.f, kTol);
    }
}
