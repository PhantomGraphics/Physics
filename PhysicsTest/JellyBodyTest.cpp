#include "pch.h"
#include <cmath>
#include "../Physics/JellyBody.h"
#include "../Physics/XPBDSolver.h"

using namespace Phantom::Physics;

// ------------------------------------------------- particle / tetra counts ---

TEST(JellyBodyTest, Build_ParticleCount) {
    JellyBodyParams p;
    p.nx = 2; p.ny = 2; p.nz = 2;
    JellyBody jelly(p);
    EXPECT_EQ(static_cast<int>(jelly.getMesh().particles.size()), 3 * 3 * 3);
}

TEST(JellyBodyTest, Build_TetraCount) {
    JellyBodyParams p;
    p.nx = 2; p.ny = 2; p.nz = 2;
    JellyBody jelly(p);
    EXPECT_EQ(static_cast<int>(jelly.getMesh().tetrahedra.size()), 2 * 2 * 2 * 5);
}

// ------------------------------------------------- constraint counts  --------

TEST(JellyBodyTest, Build_ConstraintCount) {
    JellyBodyParams p;
    p.nx = 2; p.ny = 2; p.nz = 2;
    JellyBody jelly(p);
    XPBDSolver solver;
    solver.setMesh(&jelly.getMesh());
    jelly.build(solver);

    int expectedDist = static_cast<int>(jelly.getMesh().edges.size());
    int expectedVol  = static_cast<int>(jelly.getMesh().tetrahedra.size());
    ASSERT_GT(expectedDist, 0);
    EXPECT_EQ(static_cast<int>(solver.getConstraintCount()), expectedDist + expectedVol);
}

// ------------------------------------------------- offset  -------------------

TEST(JellyBodyTest, Offset_AppliedToParticles) {
    JellyBodyParams p;
    p.nx = 1; p.ny = 1; p.nz = 1;
    p.offset = { 0.f, 2.f, 0.f };
    JellyBody jelly(p);

    float minY = jelly.getMesh().particles.positions[0].y;
    for (const auto& pos : jelly.getMesh().particles.positions)
        minY = std::min(minY, pos.y);

    // 底面 (local y=0) は offset.y までシフトしているはず
    EXPECT_NEAR(minY, 2.f, 1e-5f);
}

// NOTE: VolumeConserved_AfterFalling (100-step solver.step() loop) moved to
// Physics/PhysicsView/scenarios/jelly_drop.json, which checks
// GetSoftTotalVolume:0 before/after Step:200 on the JellyDrop preset.
