#include "pch.h"
#include "../Physics/ClothBody.h"
#include "../Physics/XPBDSolver.h"

using namespace Phantom::Physics;

namespace {

// Expected BendConstraint count for rows x cols grid:
//   Type1 (within-quad diagonal) : (rows-1)*(cols-1)
//   Type2 (vertical adjacent)    : (rows-2)*(cols-1)
//   Type3 (horizontal adjacent)  : (rows-1)*(cols-2)
int expectedBendCount(int rows, int cols) {
    return (rows-1)*(cols-1) + (rows-2)*(cols-1) + (rows-1)*(cols-2);
}

int expectedDistCount(int rows, int cols) {
    return rows * (cols - 1) + (rows - 1) * cols;  // structural edges only
}

} // namespace

// ------------------------------------------------- particle count  -----------

TEST(ClothBodyTest, Build_ParticleCount) {
    ClothBodyParams p;
    p.rows = 5; p.cols = 6;
    ClothBody cloth(p);
    EXPECT_EQ(static_cast<int>(cloth.getMesh().particles.size()), 5 * 6);
}

// ------------------------------------------------- constraint counts  --------

TEST(ClothBodyTest, Build_DistanceConstraintCount) {
    ClothBodyParams p;
    p.rows = 5; p.cols = 6;
    p.pinTopLeft = false; p.pinTopRight = false;

    ClothBody cloth(p);
    XPBDSolver solver;
    solver.setMesh(&cloth.getMesh());
    cloth.build(solver);

    int expected = expectedDistCount(p.rows, p.cols);
    // Total constraints = dist + bend; subtract bend to get dist count
    int bend = expectedBendCount(p.rows, p.cols);
    int total = static_cast<int>(solver.getConstraintCount());
    EXPECT_EQ(total - bend, expected);
}

TEST(ClothBodyTest, Build_BendConstraintCount) {
    ClothBodyParams p;
    p.rows = 5; p.cols = 6;
    p.pinTopLeft = false; p.pinTopRight = false;

    ClothBody cloth(p);
    XPBDSolver solver;
    solver.setMesh(&cloth.getMesh());
    cloth.build(solver);

    int expected = expectedBendCount(p.rows, p.cols);
    int dist     = expectedDistCount(p.rows, p.cols);
    int total    = static_cast<int>(solver.getConstraintCount());
    EXPECT_EQ(total - dist, expected);
}

// NOTE: PinnedTopCorners_DoNotMove, PinnedTopEdge_AllTopFixed, and
// Hanging_BottomFalls (100-step solver.step() loops) moved to
// Physics/PhysicsView/scenarios/cloth_two_pins.json and
// scenarios/cloth_top_edge.json, using GetSoftParticlePositionY: for the
// pinned-position checks and GetSoftMinPositionY for the bottom-falls check.

// ------------------------------------------------- getStretch initial  -------

TEST(ClothBodyTest, GetStretch_InitiallyOne) {
    ClothBodyParams p;
    p.rows = 4; p.cols = 4;
    ClothBody cloth(p);
    XPBDSolver solver;
    solver.setMesh(&cloth.getMesh());
    cloth.build(solver);

    int n = static_cast<int>(cloth.getMesh().edges.size());
    for (int i = 0; i < n; ++i)
        EXPECT_NEAR(cloth.getStretch(i), 1.f, 1e-5f);
}
