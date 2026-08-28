#include "pch.h"
#include "../Physics/SoftParticle.h"
#include "../Physics/SphereCollider.h"
#include "../Physics/PlaneCollider.h"

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

// ------------------------------------------------- SphereCollider  -----------

TEST(SphereColliderTest, ParticleOutside_Unchanged) {
    SphereCollider col;
    col.center = {0.f, 0.f, 0.f};
    col.radius = 1.f;

    SoftParticleSoA p = makeParticleSoA({0.f, 2.f, 0.f});  // outside
    col.resolve(p, 0);

    EXPECT_NEAR(p.predicted[0].y, 2.f, 1e-5f);
}

TEST(SphereColliderTest, ParticleInside_PushedOut) {
    SphereCollider col;
    col.center = {0.f, 0.f, 0.f};
    col.radius = 1.f;

    SoftParticleSoA p = makeParticleSoA({0.f, 0.5f, 0.f});  // inside
    col.resolve(p, 0);

    float dist = glm::length(p.predicted[0] - col.center);
    EXPECT_GE(dist, col.radius - 1e-5f);
}

TEST(SphereColliderTest, ParticleOnSurface_Unchanged) {
    SphereCollider col;
    col.center = {0.f, 0.f, 0.f};
    col.radius = 1.f;

    SoftParticleSoA p = makeParticleSoA({0.f, 1.f, 0.f});  // exactly on surface
    col.resolve(p, 0);

    float dist = glm::length(p.predicted[0] - col.center);
    EXPECT_NEAR(dist, 1.f, 1e-5f);
}

TEST(SphereColliderTest, OffCenterSphere_PushedOut) {
    SphereCollider col;
    col.center = {1.f, 0.f, 0.f};
    col.radius = 0.5f;

    // Particle inside sphere but NOT at exact center (avoid degenerate zero-length direction)
    SoftParticleSoA p = makeParticleSoA({1.2f, 0.f, 0.f});  // dist=0.2 < radius=0.5
    col.resolve(p, 0);

    float dist = glm::length(p.predicted[0] - col.center);
    EXPECT_GE(dist, col.radius - 1e-5f);
}

// ------------------------------------------------- PlaneCollider  ------------

TEST(PlaneColliderTest, ParticleAbove_Unchanged) {
    PlaneCollider col;
    col.normal = {0.f, 1.f, 0.f};
    col.offset = 0.f;

    SoftParticleSoA p = makeParticleSoA({0.f, 1.f, 0.f});  // above y=0
    col.resolve(p, 0);

    EXPECT_NEAR(p.predicted[0].y, 1.f, 1e-5f);
}

TEST(PlaneColliderTest, ParticleBelow_PushedUp) {
    PlaneCollider col;
    col.normal = {0.f, 1.f, 0.f};
    col.offset = 0.f;

    SoftParticleSoA p = makeParticleSoA({0.f, -0.5f, 0.f});  // below y=0
    col.resolve(p, 0);

    float signed_dist = glm::dot(p.predicted[0], col.normal) - col.offset;
    EXPECT_GE(signed_dist, -1e-5f);
}

TEST(PlaneColliderTest, ParticleOnPlane_Unchanged) {
    PlaneCollider col;
    col.normal = {0.f, 1.f, 0.f};
    col.offset = 0.f;

    SoftParticleSoA p = makeParticleSoA({3.f, 0.f, -2.f});  // on y=0
    col.resolve(p, 0);

    EXPECT_NEAR(p.predicted[0].y, 0.f, 1e-5f);
}

TEST(PlaneColliderTest, TiltedPlane_PushedOut) {
    PlaneCollider col;
    col.normal = glm::normalize(Vector3df{1.f, 1.f, 0.f});
    col.offset = 0.f;

    SoftParticleSoA p = makeParticleSoA({-1.f, -1.f, 0.f});  // on negative side
    col.resolve(p, 0);

    float signed_dist = glm::dot(p.predicted[0], col.normal) - col.offset;
    EXPECT_GE(signed_dist, -1e-5f);
}
