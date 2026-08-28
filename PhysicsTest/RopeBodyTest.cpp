#include "pch.h"
#include "../Physics/RopeBody.h"
#include "../Physics/XPBDSolver.h"

using namespace Phantom::Physics;

// ------------------------------------------------- particle count  -----------

TEST(RopeBodyTest, Build_ParticleCount) {
    RopeBodyParams p;
    p.n = 15;
    RopeBody rope(p);
    EXPECT_EQ(static_cast<int>(rope.getMesh().particles.size()), 15);
}

// ------------------------------------------------- constraint count  ---------

TEST(RopeBodyTest, Build_ConstraintCount) {
    RopeBodyParams p;
    p.n        = 10;
    p.pinStart = false;
    p.pinEnd   = false;

    RopeBody rope(p);
    XPBDSolver solver;
    solver.setMesh(&rope.getMesh());
    rope.build(solver);

    // (n-1) structural + (n-2) bending
    int expected = (p.n - 1) + (p.n - 2);
    EXPECT_EQ(static_cast<int>(solver.getConstraintCount()), expected);
}

// NOTE: BothEndsPinned_NeitherMoves (100-step solver.step() loop) moved to
// Physics/PhysicsView/scenarios/rope_both_ends_pinned.json, which uses the
// new SoftBodyPreset::RopeBothEndsPinned + GetSoftParticlePositionY checks.

// NOTE: PinnedStart_DoesNotMove and Pendulum_TipFalls (100/120-step loops)
// moved to scenarios/rope_hanging.json and scenarios/rope_pendulum.json.
