#include "pch.h"
#include "../Physics/SoftMesh.h"
#include "../Physics/XPBDSolver.h"
#include "../Physics/DistanceConstraint.h"

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {

SoftMesh makeSingleParticle(Vector3df pos, float invMass = 1.f) {
    SoftMesh mesh;
    SoftParticle p;
    p.position    = pos;
    p.predicted   = pos;
    p.velocity    = {};
    p.force       = {};
    p.inverseMass = invMass;
    mesh.particles.push_back(p);
    return mesh;
}

SoftMesh makeTwoParticles(Vector3df posA, Vector3df posB) {
    SoftMesh mesh;
    SoftParticle p;
    p.velocity = {}; p.force = {}; p.inverseMass = 1.f;
    p.position = posA; p.predicted = posA; mesh.particles.push_back(p);
    p.position = posB; p.predicted = posB; mesh.particles.push_back(p);
    return mesh;
}

} // namespace

// ------------------------------------------------- pinned particle  ----------

TEST(XPBDSolverTest, PinnedParticle_DoesNotMove) {
    SoftMesh mesh = makeSingleParticle({0.f, 1.f, 0.f}, 0.f);  // pinned

    XPBDSolver solver;
    solver.setMesh(&mesh);

    for (int i = 0; i < 100; ++i) solver.step();

    EXPECT_NEAR(mesh.particles.positions[0].x, 0.f, 1e-5f);
    EXPECT_NEAR(mesh.particles.positions[0].y, 1.f, 1e-5f);
    EXPECT_NEAR(mesh.particles.positions[0].z, 0.f, 1e-5f);
}

// ------------------------------------------------- free falling particle -----

TEST(XPBDSolverTest, FallingParticle_YDecreases) {
    SoftMesh mesh = makeSingleParticle({0.f, 2.f, 0.f}, 1.f);

    XPBDSolver solver;
    solver.setMesh(&mesh);
    // ~1 second of simulation (60 steps * 0.016)
    for (int i = 0; i < 60; ++i) solver.step();

    EXPECT_LT(mesh.particles.positions[0].y, 1.f);
}

// ------------------------------------------------- distance constraint  ------

TEST(XPBDSolverTest, DistanceConstraint_MaintainsLength) {
    // Two particles 2.0 apart, restLength=1.0, alpha=0 (rigid)
    SoftMesh mesh = makeTwoParticles({0.f, 0.f, 0.f}, {2.f, 0.f, 0.f});

    XPBDSolver solver;
    solver.params().gravity = {0.f, 0.f, 0.f};
    solver.setMesh(&mesh);

    auto dc = std::make_unique<DistanceConstraint>();
    dc->alpha      = 0.f;
    dc->a          = 0;
    dc->b          = 1;
    dc->restLength = 1.f;
    solver.addConstraint(std::move(dc));

    solver.step();

    float d = glm::length(mesh.particles.positions[1] - mesh.particles.positions[0]);
    EXPECT_NEAR(d, 1.f, 1e-3f);
}

// ------------------------------------------------- reset restores position ---

TEST(XPBDSolverTest, Reset_RestoresPosition) {
    SoftMesh mesh = makeSingleParticle({0.f, 5.f, 0.f}, 1.f);

    XPBDSolver solver;
    solver.setMesh(&mesh);

    for (int i = 0; i < 30; ++i) solver.step();

    float yAfterStep = mesh.particles.positions[0].y;
    EXPECT_LT(yAfterStep, 5.f);  // has fallen

    solver.reset();
    EXPECT_NEAR(mesh.particles.positions[0].y, 5.f, 1e-5f);
}

// ------------------------------------------------- constraint count  ---------

TEST(XPBDSolverTest, AddConstraint_CountIncreases) {
    SoftMesh mesh = makeTwoParticles({0.f,0.f,0.f},{1.f,0.f,0.f});

    XPBDSolver solver;
    solver.setMesh(&mesh);

    EXPECT_EQ(solver.getConstraintCount(), 0u);

    auto dc = std::make_unique<DistanceConstraint>();
    dc->a = 0; dc->b = 1; dc->restLength = 1.f;
    solver.addConstraint(std::move(dc));

    EXPECT_EQ(solver.getConstraintCount(), 1u);

    solver.clearConstraints();
    EXPECT_EQ(solver.getConstraintCount(), 0u);
}
