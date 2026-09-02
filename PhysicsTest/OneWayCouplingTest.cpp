#include "pch.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/RigidBoundary.h"
#include "../Physics/ICollisionShape.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);
}

// A fluid particle dropped above a static sphere should bounce off the SDF
// penalty boundary rather than sinking through it: the minimum distance from
// the sphere center observed over the whole fall/bounce should stay close to
// (not much less than) the sphere radius.
TEST(OneWayCouplingTest, PBSPH_ParticleFallingOntoStaticSphere_DoesNotPenetrate) {
    PBSPHFluid fluid;
    fluid.setRestDensity(1.0f);
    fluid.setStiffness(1.0f);
    fluid.setVicsosity(0.0f);
    fluid.setEffectLength(0.3f);
    fluid.setIsBoundary(false);

    fluid.createParticle(Vector3df(0.f, 0.8f, 0.f), 0.05f);

    SphereShape sphere;
    sphere.radius = 0.5f;
    RigidBoundary boundary;
    boundary.setShape(&sphere);
    boundary.setPenaltyStiffness(2000.f);
    boundary.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

    PBSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce({ 0.f, -9.8f, 0.f });
    solver.addRigidBoundary(&boundary);
    solver.setTimeStep(0.01f);

    float minDist = getLength(fluid.getParticles().positions[0]);
    constexpr int kSteps = 100;
    for (int i = 0; i < kSteps; ++i) {
        solver.simulate(0.01f, 2);
        const auto& pos = fluid.getParticles().positions[0];
        ASSERT_TRUE(std::isfinite(pos.y));
        minDist = std::min(minDist, getLength(pos));
    }

    // Allow some penalty-spring give, but nowhere near a full pass-through.
    EXPECT_GT(minDist, sphere.radius - 0.15f);
}

// internal design notes Phase 2 (#1): a particle falling
// onto a RigidBoundary plane whose stiffness comes from
// RigidBoundary::estimateStiffness(dt) should stay bounded near the plane at
// both the historical scale (radius=1) and a 10x-smaller one, with gravity
// and dt held fixed -- without a scale-aware stiffness, the same fixed
// penaltyStiffness_ would either let the small-scale particle tunnel through
// (stiffness too weak relative to gravity at that scale) or diverge
// (stiffness too strong relative to dt's explicit-integration stability
// bound), mirroring the #9 pressureCoe failure mode from section 4.
TEST(OneWayCouplingTest, PBSPH_ParticleFallingOntoStaticPlane_StiffnessFromScaleStaysBoundedAcrossScales) {
    const float dt = 0.01f;

    for (float radius : {1.0f, 0.1f}) {
        PBSPHFluid fluid;
        fluid.setRestDensity(1.0f);
        fluid.setStiffness(1.0f);
        fluid.setVicsosity(0.0f);
        fluid.setEffectLength(2.25f * radius);
        fluid.setIsBoundary(false);
        fluid.createParticle(Vector3df(0.f, 3.f * radius, 0.f), 0.1f * radius);

        PlaneShape plane;
        plane.normal = { 0.f, 1.f, 0.f };
        plane.offset = 0.f;
        RigidBoundary boundary;
        boundary.setShape(&plane);
        boundary.setStiffnessFromScale(dt);
        boundary.syncKinematic({ 0.f, 0.f, 0.f }, kIdentity);

        PBSPHSolver solver;
        solver.add(&fluid);
        solver.setExternalForce({ 0.f, -9.8f, 0.f });
        solver.addRigidBoundary(&boundary);
        solver.setTimeStep(dt);

        for (int i = 0; i < 200; ++i) {
            solver.simulate(dt, 2);
            const auto& pos = fluid.getParticles().positions[0];
            ASSERT_TRUE(std::isfinite(pos.y)) << "radius=" << radius << " step=" << i;
        }

        const auto& pos = fluid.getParticles().positions[0];
        EXPECT_NEAR(pos.y, 0.f, 2.0f * radius) << "radius=" << radius;
    }
}

// A particle initially embedded in a box (given the box's *current*
// orientation) must be pushed out along the box's rotated local frame -- not
// its unrotated one -- demonstrating that RigidBoundary's orientation is
// actually honored through the solver pipeline (RigidBoundaryTest already
// covers the raw getBoundaryForce() math in isolation).
TEST(OneWayCouplingTest, PBSPH_ParticleEmbeddedInRotatedBox_PushedOut) {
    PBSPHFluid fluid;
    fluid.setRestDensity(1.0f);
    fluid.setStiffness(1.0f);
    fluid.setVicsosity(0.0f);
    fluid.setEffectLength(0.3f);
    fluid.setIsBoundary(false);

    const Vector3df startPos(0.6f, 0.f, 0.f);
    fluid.createParticle(startPos, 0.05f);

    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };
    const Quaternion rot45 = glm::angleAxis(glm::radians(45.f), Vector3df(0.f, 0.f, 1.f));

    // Sanity check: this point sits outside an *unrotated* box (0.6 > 0.5)
    // but is embedded once the box is rotated 45 degrees about Z.
    ASSERT_LT(box.getSignedDistance(startPos, { 0.f, 0.f, 0.f }, rot45), 0.f);

    RigidBoundary boundary;
    boundary.setShape(&box);
    boundary.setPenaltyStiffness(2000.f);
    boundary.syncKinematic({ 0.f, 0.f, 0.f }, rot45);

    PBSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce({ 0.f, 0.f, 0.f });  // isolate the boundary push
    solver.addRigidBoundary(&boundary);
    solver.setTimeStep(0.01f);

    for (int i = 0; i < 5; ++i) {
        solver.simulate(0.01f, 2);
    }

    const auto& pos = fluid.getParticles().positions[0];
    EXPECT_GT(box.getSignedDistance(pos, { 0.f, 0.f, 0.f }, rot45), 0.f);
    EXPECT_GT(getLength(pos), getLength(startPos));
}
