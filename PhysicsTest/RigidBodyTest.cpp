#include "pch.h"
#include "../Physics/RigidBody.h"
#include "../Physics/RigidBodySolver.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;
}

TEST(RigidBodyTest, GravityIntegration_FallsWithExpectedVelocity) {
    RigidBodySolver world;
    world.params().gravity = { 0.f, -9.8f, 0.f };
    world.timeStep = 0.016f;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto body = std::make_unique<RigidBody>();
    body->setShape(&sphere);
    body->setMass(1.0f);

    RigidBody* b = body.get();
    world.addBody(b);
    world.saveSnapshot();
    world.setRunning(true);
    world.step();

    // v = g * dt (gravity applied before position integration)
    float expectedVy = -9.8f * 0.016f;
    EXPECT_NEAR(b->linearVelocity.y, expectedVy, kTol);
    EXPECT_NEAR(b->linearVelocity.x, 0.f, kTol);
    EXPECT_NEAR(b->linearVelocity.z, 0.f, kTol);
}

TEST(RigidBodyTest, StaticBody_DoesNotMove) {
    RigidBodySolver world;
    world.params().gravity = { 0.f, -9.8f, 0.f };
    world.timeStep = 0.016f;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto body = std::make_unique<RigidBody>();
    body->position = { 0.f, 5.f, 0.f };
    body->setShape(&sphere);
    body->setMass(0.f);  // static

    RigidBody* b = body.get();
    world.addBody(b);
    world.saveSnapshot();
    world.setRunning(true);

    for (int i = 0; i < 20; ++i)
        world.step();

    EXPECT_NEAR(b->position.x, 0.f, kTol);
    EXPECT_NEAR(b->position.y, 5.f, kTol);
    EXPECT_NEAR(b->position.z, 0.f, kTol);
    EXPECT_NEAR(b->linearVelocity.y, 0.f, kTol);
    EXPECT_NEAR(b->angularVelocity.y, 0.f, kTol);
}

TEST(RigidBodyTest, ApplyImpulse_LinearVelocityChanges) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBody body;
    body.setShape(&sphere);
    body.setMass(2.0f);
    body.updateInertiaTensor();

    // Impulse at center of mass -- no angular effect
    Vector3df impulse(0.f, 10.f, 0.f);
    body.applyImpulse(impulse, body.position);

    // dv = J / mass = 10 / 2 = 5
    EXPECT_NEAR(body.linearVelocity.y, 5.f, kTol);
    EXPECT_NEAR(body.angularVelocity.x, 0.f, kTol);
    EXPECT_NEAR(body.angularVelocity.y, 0.f, kTol);
    EXPECT_NEAR(body.angularVelocity.z, 0.f, kTol);
}

TEST(RigidBodyTest, ApplyImpulse_OffCenter_CreatesAngularVelocity) {
    SphereShape sphere;
    sphere.radius = 0.5f;

    RigidBody body;
    body.setShape(&sphere);
    body.setMass(1.0f);
    body.updateInertiaTensor();

    // Impulse at offset point -- generates torque
    // r = (0.5, 0, 0), J = (0, 1, 0) => r x J = (0, 0, 0.5)
    Vector3df worldPoint = body.position + Vector3df(0.5f, 0.f, 0.f);
    body.applyImpulse(Vector3df(0.f, 1.f, 0.f), worldPoint);

    EXPECT_GT(std::abs(body.angularVelocity.z), kTol);
}

TEST(RigidBodyTest, Reset_RestoresInitialState) {
    RigidBodySolver world;
    world.params().gravity = { 0.f, -9.8f, 0.f };
    world.timeStep = 0.016f;

    SphereShape sphere;
    sphere.radius = 0.5f;

    auto body = std::make_unique<RigidBody>();
    body->position = { 0.f, 5.f, 0.f };
    body->setShape(&sphere);
    body->setMass(1.0f);

    RigidBody* b = body.get();
    world.addBody(b);
    world.saveSnapshot();
    world.setRunning(true);

    for (int i = 0; i < 50; ++i)
        world.step();

    EXPECT_LT(b->position.y, 5.f);

    world.reset();

    EXPECT_NEAR(b->position.y, 5.f, kTol);
    EXPECT_NEAR(b->linearVelocity.y, 0.f, kTol);
}
