#include "pch.h"
#include "../Physics/RigidSoftSolver.h"
#include "../Physics/ClothBody.h"
#include "Physics/Physics/RigidBody.h"
#include "Physics/Physics/ICollisionShape.h"
#include <cmath>
#include <memory>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;

// Minimal ISoftBody with a single free particle and no constraints, used to
// isolate the RigidBodyCollider/reaction-force path from XPBD
// constraint-projection effects (mirrors SoftFluidSolverTest.cpp's
// SingleParticleBody).
class SingleParticleBody : public ISoftBody {
public:
    explicit SingleParticleBody(const Vector3df& pos) {
        mesh_.particles.resize(1);
        mesh_.particles.positions[0]    = pos;
        mesh_.particles.predicted[0]   = pos;
        mesh_.particles.inverseMasses[0] = 1.f;
    }
    void            build(XPBDSolver&)       override {}
    SoftMesh&       getMesh()                override { return mesh_; }
    const SoftMesh& getMesh()          const override { return mesh_; }

private:
    SoftMesh mesh_;
};

// One PhysicsSolver::stepUnconditional()-style frame, driven manually since
// this test bridges standalone RigidBodySolver/SoftBodySolver instances
// (mirrors RigidSoftSolver's "does not own either world" design).
void stepFrame(RigidSoftSolver& bridge, RigidBodySolver& rigidWorld, SoftBodySolver& softWorld, float dt) {
    bridge.applyTwoWayReactions(softWorld);
    rigidWorld.timeStep = dt;
    rigidWorld.stepUnconditional();
    softWorld.solverParams().timeStep = dt;
    softWorld.stepUnconditional();
}

} // namespace

// A cloth bound OneWay should still get pushed out by (never penetrate) the
// rigid box -- RigidBodyCollider's position constraint is always active,
// regardless of mode -- while the rigid body itself -- given zero gravity
// here, to isolate the coupling from its own free-fall motion -- must stay
// completely at rest, since OneWay carries no reaction.
TEST(RigidSoftSolverTest, OneWay_ClothPushedOutButRigidBodyUnaffected) {
    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBodySolver rigidWorld;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, 0.f, 0.f };
    bodyOwned->setShape(&box);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    rigidWorld.addBody(body);
    rigidWorld.params().gravity = { 0.f, 0.f, 0.f };  // isolate: OneWay must not move it regardless
    rigidWorld.saveSnapshot();

    SoftBodySolver softWorld;  // default (-9.8) gravity drives the cloth down through the box
    ClothBodyParams p;
    p.rows       = 5;
    p.cols       = 5;
    p.width      = 1.5f;
    p.height     = 1.5f;
    p.pinTopEdge = true;
    p.origin     = { 0.f, 1.f, 0.f };  // starts above the box, falls down through it
    auto clothOwned = std::make_unique<ClothBody>(p);
    auto* cloth  = clothOwned.get();
    softWorld.addBody(cloth);

    RigidSoftSolver bridge;
    bridge.bind(body, softWorld, CouplingMode::OneWay);

    for (int i = 0; i < 200; ++i) {
        stepFrame(bridge, rigidWorld, softWorld, 0.01f);
    }

    // No cloth particle should have penetrated the (still-at-origin) box.
    for (const auto& pos : cloth->getMesh().particles.positions) {
        const float d = box.getSignedDistance(pos, body->position, body->orientation);
        EXPECT_GE(d, -1.0e-3f);
    }

    EXPECT_NEAR(getDistance(body->position, Vector3df(0.f, 0.f, 0.f)), 0.f, kTol);
    EXPECT_NEAR(getLength(body->linearVelocity), 0.f, kTol);
}

// A sphere pressing down into an already-overlapping cloth from above should
// accumulate an upward Newton's-third-law reaction (the cloth resists the
// intrusion and supports the sphere, mirrors TwoWayCouplingTest.cpp's
// PBSPH_BallEnteringPuddle_VelocityDampedRelativeToFreeFall), decelerating it
// relative to a free-falling control with no coupling at all.
TEST(RigidSoftSolverTest, TwoWay_FallingRigidBody_DeceleratesOnPinnedCloth) {
    SoftBodySolver softWorld;
    softWorld.solverParams().gravity = { 0.f, 0.f, 0.f };  // isolate the coupling force from the cloth's own weight

    ClothBodyParams p;
    p.rows       = 7;
    p.cols       = 7;
    p.width      = 2.f;
    p.height     = 2.f;
    p.pinTopEdge = true;
    auto clothOwned = std::make_unique<ClothBody>(p);
    softWorld.addBody(clothOwned.get());

    RigidBodySolver rigidWorld;
    SphereShape sphere;
    sphere.radius = 0.3f;

    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, 0.15f, 0.f };  // embedded 0.15 into the cloth (y=0 plane) from above
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.0f);
    RigidBody* body = bodyOwned.get();
    rigidWorld.addBody(body);
    rigidWorld.params().gravity = { 0.f, -9.8f, 0.f };
    rigidWorld.saveSnapshot();

    RigidSoftSolver bridge;
    bridge.bind(body, softWorld, CouplingMode::TwoWay);

    const float dt = 0.005f;
    for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(std::isfinite(body->position.y));
        stepFrame(bridge, rigidWorld, softWorld, dt);
    }

    // Free-fall control over the same duration, with no soft-body coupling.
    const float freeFallVy = -9.8f * dt * 10.f;
    EXPECT_GT(body->linearVelocity.y, freeFallVy);
}

// Archimedes-like comparison mirroring
// TwoWayCouplingTest.cpp's PBSPH_LighterSphereSinksLessThanHeavierSphere:
// given the same reaction support from the cloth (same shape, same starting
// depth), a lighter body should sink less than a heavier one.
TEST(RigidSoftSolverTest, TwoWay_HeavierRigidBody_PenetratesFartherThanLighter) {
    auto runDrop = [](float mass) {
        SoftBodySolver softWorld;
        softWorld.solverParams().gravity = { 0.f, 0.f, 0.f };

        ClothBodyParams p;
        p.rows       = 7;
        p.cols       = 7;
        p.width      = 2.f;
        p.height     = 2.f;
        p.pinTopEdge = true;
        auto clothOwned = std::make_unique<ClothBody>(p);
        softWorld.addBody(clothOwned.get());

        RigidBodySolver rigidWorld;
        SphereShape sphere;
        sphere.radius = 0.3f;

        auto bodyOwned = std::make_unique<RigidBody>();
        bodyOwned->position = { 0.f, 0.15f, 0.f };  // embedded into the cloth from above
        bodyOwned->setShape(&sphere);
        bodyOwned->setMass(mass);
        RigidBody* body = bodyOwned.get();
    rigidWorld.addBody(body);
        rigidWorld.params().gravity = { 0.f, -9.8f, 0.f };
        rigidWorld.saveSnapshot();

        RigidSoftSolver bridge;
        bridge.bind(body, softWorld, CouplingMode::TwoWay);

        const float dt = 0.005f;
        for (int i = 0; i < 10; ++i) {
            stepFrame(bridge, rigidWorld, softWorld, dt);
        }
        return body->position.y;
    };

    const float lightFinalY = runDrop(0.2f);
    const float heavyFinalY = runDrop(5.0f);

    EXPECT_GT(lightFinalY, heavyFinalY);
}

// After clearBindings(), the rigid box's collider must no longer resolve the
// soft particle (it can now overlap freely -- "pass through"), and the rigid
// body's velocity must stop receiving any reaction.
TEST(RigidSoftSolverTest, ClearBindings_StopsBothCollisionAndReaction) {
    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBodySolver rigidWorld;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, 5.f, 0.f };
    bodyOwned->setShape(&box);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    rigidWorld.addBody(body);
    rigidWorld.params().gravity = { 0.f, 0.f, 0.f };  // isolate: only the reaction should move it
    rigidWorld.saveSnapshot();

    SoftBodySolver softWorld;
    softWorld.solverParams().gravity = { 0.f, 0.f, 0.f };
    auto particleOwned = std::make_unique<SingleParticleBody>(Vector3df(0.f, 5.3f, 0.f));  // embedded 0.2 inside the box, above its center
    auto* particleBody = particleOwned.get();
    softWorld.addBody(particleBody);

    RigidSoftSolver bridge;
    bridge.bind(body, softWorld, CouplingMode::TwoWay);

    stepFrame(bridge, rigidWorld, softWorld, 0.01f);

    const float vyBound = body->linearVelocity.y;
    // The particle (embedded above the box's center) is pushed further up and
    // out by RigidBodyCollider; by Newton's third law the (gravity-free) box
    // feels the opposite reaction and gets pushed down.
    EXPECT_LT(vyBound, 0.f);

    const float distBound = box.getSignedDistance(
        particleBody->getMesh().particles.positions[0], body->position, body->orientation);
    EXPECT_GE(distBound, -1.0e-3f);  // corrected back onto (or outside) the surface, not penetrating

    bridge.clearBindings(softWorld);

    // Re-embed the particle (position AND velocity) to prove collision no
    // longer resolves it once the collider registration has been cleared.
    particleBody->getMesh().particles.positions[0]  = body->position;
    particleBody->getMesh().particles.predicted[0]  = body->position;
    particleBody->getMesh().particles.velocities[0]  = { 0.f, 0.f, 0.f };

    stepFrame(bridge, rigidWorld, softWorld, 0.01f);

    // No reaction anymore: velocity is unchanged (no force, no gravity).
    EXPECT_NEAR(body->linearVelocity.y, vyBound, kTol);

    // No collision anymore: the particle is left deep inside the box.
    const float distAfterClear = box.getSignedDistance(
        particleBody->getMesh().particles.positions[0], body->position, body->orientation);
    EXPECT_LT(distAfterClear, -0.1f);
}
