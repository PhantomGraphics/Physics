#include "pch.h"
#include "../Physics/RigidBody.h"
#include "../Physics/NarrowPhase.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;

void setupSphere(RigidBody& b, const Vector3df& pos, float r, float mass = 1.f) {
    // shape stored in test-local storage; caller is responsible for lifetime
    b.position = pos;
}
}

// ------------------------------------------------ Sphere-Sphere --------

TEST(CollisionTest, SphereSphere_Overlap_DetectsContact) {
    SphereShape sa, sb;
    sa.radius = 0.5f;
    sb.radius = 0.5f;

    RigidBody a, b;
    a.position = { 0.f, 0.f, 0.f };
    b.position = { 0.8f, 0.f, 0.f };  // overlap = 0.2
    a.setShape(&sa); a.setMass(1.f);
    b.setShape(&sb); b.setMass(1.f);

    ContactManifold out;
    bool hit = NarrowPhase::detect(a, b, out);

    EXPECT_TRUE(hit);
    ASSERT_EQ(out.contacts.size(), 1u);
    EXPECT_NEAR(out.contacts[0].penetration, 0.2f, kTol);
    // normal should point from b toward a (i.e., in -x direction)
    EXPECT_LT(out.contacts[0].normal.x, 0.f);
}

TEST(CollisionTest, SphereSphere_NoOverlap_NoContact) {
    SphereShape sa, sb;
    sa.radius = 0.5f;
    sb.radius = 0.5f;

    RigidBody a, b;
    a.position = { 0.f, 0.f, 0.f };
    b.position = { 2.f, 0.f, 0.f };  // gap = 1.0
    a.setShape(&sa); a.setMass(1.f);
    b.setShape(&sb); b.setMass(1.f);

    ContactManifold out;
    bool hit = NarrowPhase::detect(a, b, out);

    EXPECT_FALSE(hit);
    EXPECT_TRUE(out.contacts.empty());
}

// ------------------------------------------------ Sphere-Plane --------

TEST(CollisionTest, SpherePlane_BelowPlane_DetectsContact) {
    SphereShape ss;
    ss.radius = 0.5f;
    PlaneShape ps;
    ps.normal = { 0.f, 1.f, 0.f };
    ps.offset = 0.f;

    RigidBody a, b;  // a = sphere, b = plane
    a.position = { 0.f, 0.3f, 0.f };  // sphere bottom at y=-0.2 => penetrating
    b.position = { 0.f, 0.f, 0.f };
    a.setShape(&ss); a.setMass(1.f);
    b.setShape(&ps); b.setMass(0.f);

    ContactManifold out;
    bool hit = NarrowPhase::detect(a, b, out);

    EXPECT_TRUE(hit);
    ASSERT_FALSE(out.contacts.empty());
    EXPECT_NEAR(out.contacts[0].penetration, 0.2f, kTol);
    EXPECT_NEAR(out.contacts[0].normal.y, 1.f, kTol);  // pushes sphere up
}

TEST(CollisionTest, SpherePlane_Above_NoContact) {
    SphereShape ss;
    ss.radius = 0.5f;
    PlaneShape ps;
    ps.normal = { 0.f, 1.f, 0.f };
    ps.offset = 0.f;

    RigidBody a, b;
    a.position = { 0.f, 1.f, 0.f };  // sphere bottom at y=0.5 => no contact
    b.position = { 0.f, 0.f, 0.f };
    a.setShape(&ss); a.setMass(1.f);
    b.setShape(&ps); b.setMass(0.f);

    ContactManifold out;
    EXPECT_FALSE(NarrowPhase::detect(a, b, out));
}

// ------------------------------------------------ Sphere-Box ----------

TEST(CollisionTest, SphereBox_Inside_DetectsContact) {
    SphereShape ss;
    ss.radius = 0.5f;
    BoxShape bs;
    bs.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBody a, b;  // a = sphere, b = box
    a.position = { 0.7f, 0.f, 0.f };  // sphere center at x=0.7, box extends to x=0.5 => overlap 0.3
    b.position = { 0.f, 0.f, 0.f };
    a.setShape(&ss); a.setMass(1.f);
    b.setShape(&bs); b.setMass(1.f);

    ContactManifold out;
    bool hit = NarrowPhase::detect(a, b, out);

    EXPECT_TRUE(hit);
    ASSERT_FALSE(out.contacts.empty());
    EXPECT_NEAR(out.contacts[0].penetration, 0.3f, kTol);
    // normal points from box (b) toward sphere (a): +x direction
    EXPECT_GT(out.contacts[0].normal.x, 0.f);
}

TEST(CollisionTest, SphereBox_Separated_NoContact) {
    SphereShape ss;
    ss.radius = 0.5f;
    BoxShape bs;
    bs.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBody a, b;
    a.position = { 2.f, 0.f, 0.f };  // far from box
    b.position = { 0.f, 0.f, 0.f };
    a.setShape(&ss); a.setMass(1.f);
    b.setShape(&bs); b.setMass(1.f);

    ContactManifold out;
    EXPECT_FALSE(NarrowPhase::detect(a, b, out));
}

// ------------------------------------------------ Box-Plane -----------

TEST(CollisionTest, BoxPlane_CornerBelow_DetectsContact) {
    BoxShape bs;
    bs.halfExtents = { 0.5f, 0.5f, 0.5f };
    PlaneShape ps;
    ps.normal = { 0.f, 1.f, 0.f };
    ps.offset = 0.f;

    RigidBody a, b;  // a = box, b = plane
    a.position = { 0.f, 0.3f, 0.f };  // box bottom at y=-0.2 => 4 corners penetrate
    b.position = { 0.f, 0.f, 0.f };
    a.setShape(&bs); a.setMass(1.f);
    b.setShape(&ps); b.setMass(0.f);

    ContactManifold out;
    bool hit = NarrowPhase::detect(a, b, out);

    EXPECT_TRUE(hit);
    EXPECT_EQ(out.contacts.size(), 4u);  // all 4 bottom corners
    for (const auto& cp : out.contacts) {
        EXPECT_NEAR(cp.penetration, 0.2f, kTol);
        EXPECT_NEAR(cp.normal.y, 1.f, kTol);
    }
}

TEST(CollisionTest, BoxPlane_Above_NoContact) {
    BoxShape bs;
    bs.halfExtents = { 0.5f, 0.5f, 0.5f };
    PlaneShape ps;
    ps.normal = { 0.f, 1.f, 0.f };
    ps.offset = 0.f;

    RigidBody a, b;
    a.position = { 0.f, 1.f, 0.f };  // box bottom at y=0.5 => no contact
    b.position = { 0.f, 0.f, 0.f };
    a.setShape(&bs); a.setMass(1.f);
    b.setShape(&ps); b.setMass(0.f);

    ContactManifold out;
    EXPECT_FALSE(NarrowPhase::detect(a, b, out));
}

// ------------------------------------------------ Box-Box (SAT) -------

TEST(CollisionTest, BoxBox_Overlap_SAT_DetectsContact) {
    BoxShape sa, sb;
    sa.halfExtents = { 0.5f, 0.5f, 0.5f };
    sb.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBody a, b;
    a.position = { 0.f, 0.f, 0.f };
    b.position = { 0.8f, 0.f, 0.f };  // overlap = 0.2 in x
    a.setShape(&sa); a.setMass(1.f);
    b.setShape(&sb); b.setMass(1.f);

    ContactManifold out;
    bool hit = NarrowPhase::detect(a, b, out);

    EXPECT_TRUE(hit);
    ASSERT_FALSE(out.contacts.empty());
    EXPECT_NEAR(out.contacts[0].penetration, 0.2f, kTol);
    // normal points from b toward a (-x direction)
    EXPECT_LT(out.contacts[0].normal.x, 0.f);
}

TEST(CollisionTest, BoxBox_Separated_NoContact) {
    BoxShape sa, sb;
    sa.halfExtents = { 0.5f, 0.5f, 0.5f };
    sb.halfExtents = { 0.5f, 0.5f, 0.5f };

    RigidBody a, b;
    a.position = { 0.f, 0.f, 0.f };
    b.position = { 3.f, 0.f, 0.f };  // gap = 2
    a.setShape(&sa); a.setMass(1.f);
    b.setShape(&sb); b.setMass(1.f);

    ContactManifold out;
    EXPECT_FALSE(NarrowPhase::detect(a, b, out));
}
