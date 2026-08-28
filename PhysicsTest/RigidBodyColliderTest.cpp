#include "pch.h"
#include "../Physics/RigidBodyCollider.h"
#include "../Physics/SoftBodySolver.h"
#include "../Physics/ClothBody.h"
#include "Physics/Physics/RigidBody.h"
#include "Physics/Physics/ICollisionShape.h"

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {

SoftParticleSoA makeParticleSoA(Vector3df pos) {
    SoftParticle p;
    p.position    = pos;
    p.predicted   = pos;
    p.velocity    = {};
    p.force       = {};
    p.inverseMass = 1.f;
    SoftParticleSoA soa;
    soa.push_back(p);
    return soa;
}

} // namespace

// ------------------------------------------------- RigidBodyCollider (sphere) -

TEST(RigidBodyColliderTest, SphereShape_ParticleOutside_Unchanged) {
    SphereShape shape;
    shape.radius = 1.f;

    RigidBody body;
    body.position = { 0.f, 0.f, 0.f };
    body.setShape(&shape);

    RigidBodyCollider col(&body);
    SoftParticleSoA p = makeParticleSoA({ 0.f, 2.f, 0.f });  // outside
    col.resolve(p, 0);

    EXPECT_NEAR(p.predicted[0].y, 2.f, 1e-5f);
}

TEST(RigidBodyColliderTest, SphereShape_ParticleInside_PushedToSurface) {
    SphereShape shape;
    shape.radius = 1.f;

    RigidBody body;
    body.position = { 0.f, 0.f, 0.f };
    body.setShape(&shape);

    RigidBodyCollider col(&body);
    SoftParticleSoA p = makeParticleSoA({ 0.f, 0.5f, 0.f });  // inside
    col.resolve(p, 0);

    float dist = glm::length(p.predicted[0] - body.position);
    EXPECT_NEAR(dist, 1.f, 1e-4f);
}

TEST(RigidBodyColliderTest, SphereShape_MovedBody_UsesCurrentPosition) {
    SphereShape shape;
    shape.radius = 0.5f;

    RigidBody body;
    body.position = { 2.f, 0.f, 0.f };  // moved from origin
    body.setShape(&shape);

    RigidBodyCollider col(&body);
    SoftParticleSoA p = makeParticleSoA({ 2.2f, 0.f, 0.f });  // inside moved sphere
    col.resolve(p, 0);

    float dist = glm::length(p.predicted[0] - body.position);
    EXPECT_NEAR(dist, 0.5f, 1e-4f);
}

// ------------------------------------------------- RigidBodyCollider (box) ----

TEST(RigidBodyColliderTest, BoxShape_ParticleInside_PushedOutNearestFace) {
    BoxShape shape;
    shape.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBody body;
    body.position = { 0.f, 0.f, 0.f };
    body.setShape(&shape);

    RigidBodyCollider col(&body);
    SoftParticleSoA p = makeParticleSoA({ 0.f, 0.4f, 0.f });  // inside, closest to +Y face
    col.resolve(p, 0);

    EXPECT_NEAR(p.predicted[0].y, 0.5f, 1e-4f);
}

// NOTE: SoftBodyWorld_ClothRestsOnRigidBox (200-step world.step() loop) moved
// to Physics/PhysicsView/scenarios/cloth_on_box.json, which uses the new
// SoftBodyPreset::ClothOnBox + GetSoftMinPositionY.
