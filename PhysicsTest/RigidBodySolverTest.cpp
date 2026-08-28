#include "pch.h"
#include "../Physics/RigidBody.h"
#include "../Physics/RigidBodySolver.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

// --------------------------------------------------------------- helpers ----

namespace {

using BodyPool = std::vector<std::unique_ptr<RigidBody>>;

// Returns the plane body pointer.  Plane shape lifetime must outlive world.
// RigidBodySolver only holds a non-owning pointer, so the body itself is kept alive in `pool`
// (which must outlive `world`'s use).
RigidBody* addPlane(RigidBodySolver& world, BodyPool& pool, PlaneShape& shape,
                    float restitution = 0.f, float friction = 0.5f) {
    auto body = std::make_unique<RigidBody>();
    body->setShape(&shape);
    body->setMass(0.f);
    body->restitution = restitution;
    body->friction    = friction;
    RigidBody* ptr = body.get();
    pool.push_back(std::move(body));
    world.addBody(ptr);
    return ptr;
}

RigidBody* addSphere(RigidBodySolver& world, BodyPool& pool, SphereShape& shape,
                     const Vector3df& pos, const Vector3df& vel,
                     float mass, float restitution = 0.f, float friction = 0.5f) {
    auto body = std::make_unique<RigidBody>();
    body->setShape(&shape);
    body->setMass(mass);
    body->position       = pos;
    body->linearVelocity = vel;
    body->restitution    = restitution;
    body->friction       = friction;
    RigidBody* ptr = body.get();
    pool.push_back(std::move(body));
    world.addBody(ptr);
    return ptr;
}

RigidBody* addBox(RigidBodySolver& world, BodyPool& pool, BoxShape& shape,
                  const Vector3df& pos, float mass,
                  float restitution = 0.f, float friction = 0.3f) {
    auto body = std::make_unique<RigidBody>();
    body->setShape(&shape);
    body->setMass(mass);
    body->position    = pos;
    body->restitution = restitution;
    body->friction    = friction;
    RigidBody* ptr = body.get();
    pool.push_back(std::move(body));
    world.addBody(ptr);
    return ptr;
}

} // namespace

// ----------------------------------------- SpherePlane bounce (e=1) --------

TEST(RigidBodySolverTest, SpherePlane_Bounce_VelocityReversed) {
    RigidBodySolver world;
    world.params().gravity        = { 0.f, 0.f, 0.f };  // no gravity
    world.params().solverIterations = 20;
    world.timeStep = 0.016f;

    PlaneShape floorShape;
    floorShape.normal = { 0.f, 1.f, 0.f };
    floorShape.offset = 0.f;

    SphereShape sphereShape;
    sphereShape.radius = 0.5f;

    BodyPool pool;
    addPlane(world, pool, floorShape, 1.0f, 0.f);
    // Sphere center at y=0.51, moving down at -3 m/s.
    // After one step: center at 0.51-3*0.016=0.462, penetration = 0.5-0.462 = 0.038.
    RigidBody* sph = addSphere(world, pool, sphereShape,
                               { 0.f, 0.51f, 0.f }, { 0.f, -3.f, 0.f },
                               1.f, 1.0f, 0.f);

    world.saveSnapshot();
    world.setRunning(true);
    world.step();

    // With e=1 the impulse reverses the velocity from -3 to positive.
    EXPECT_GT(sph->linearVelocity.y, 0.f);
}

// NOTE: SpherePlane_Rest_VelocityNearZero (300-step loop) moved to
// Physics/PhysicsView/scenarios/sphere_drop.json (its SphereDrop preset uses
// this exact floor+sphere setup), which now checks GetBodyPositionY:1 /
// GetBodyVelocityY:1 bounds after enough steps to settle.

// NOTE: BoxStack_ThreeCubes_Stable (300-step loop) moved to
// Physics/PhysicsView/scenarios/stacking.json (its Stacking preset stacks 5
// boxes instead of 3, same physics), which now checks GetBodyPositionY:idx /
// GetBodyVelocityY:idx bounds after Step:300.
