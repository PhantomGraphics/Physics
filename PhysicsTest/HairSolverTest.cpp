#include "pch.h"
#include "../Physics/HairGenerator.h"
#include "../Physics/HairSolver.h"
#include <limits>

using namespace Phantom::Physics;
using Phantom::Math::Vector3df;

namespace {
HairGeneratorParams single() {
    HairGeneratorParams p;
    p.strands = 1;
    p.particlesPerStrand = 16;
    p.length = 0.6f;
    return p;
}
}

TEST(HairGeneratorTest, DeterministicBundleAndIndependentRoots) {
    HairStrands a, b;
    HairGeneratorParams p;
    ASSERT_TRUE(generateHairBundle(a, p));
    ASSERT_TRUE(generateHairBundle(b, p));
    EXPECT_EQ(a.strandCount(), 48u);
    EXPECT_EQ(a.particleCount(), 48u*24u);
    for (size_t i = 0; i < a.particleCount(); ++i)
        EXPECT_FLOAT_EQ(glm::length(a.particles().positions[i] - b.particles().positions[i]), 0.f);
    for (const auto& r : a.ranges()) {
        EXPECT_EQ(a.particles().inverseMasses[r.offset], 0.f);
        EXPECT_FLOAT_EQ(glm::length(a.particles().positions[r.offset] - r.root.position), 0.f);
        EXPECT_GT(a.particles().inverseMasses[r.offset+1], 0.f);
        float arcLength = 0.f;
        for (size_t j = 1; j < r.count; ++j)
            arcLength += glm::length(a.restPositions()[r.offset+j] - a.restPositions()[r.offset+j-1]);
        EXPECT_NEAR(arcLength, p.length, 1.e-5f);
    }
}

TEST(HairGeneratorTest, RejectsInvalidInputWithoutChangingOutput) {
    HairStrands hair;
    auto p = single();
    ASSERT_TRUE(generateHairBundle(hair, p));
    const auto revision = hair.revision();
    p.particlesPerStrand = 1;
    EXPECT_FALSE(generateHairBundle(hair, p));
    p = single(); p.length = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(generateHairBundle(hair, p));
    p = single(); p.root.rotation = {0.f, 0.f, 0.f, 0.f};
    EXPECT_FALSE(generateHairBundle(hair, p));
    EXPECT_EQ(hair.revision(), revision);
    EXPECT_EQ(hair.particleCount(), 16u);
}

TEST(HairGeneratorTest, ValidatesStrandGeometryAndNormalizesRotation) {
    HairStrands hair;
    HairStrandInput in;
    in.root.rotation = {2.f, 0.f, 0.f, 0.f};
    in.restPositions = {{0.f, 0.f, 0.f}, {0.f, -1.f, 0.f}};
    ASSERT_TRUE(hair.initialize({in}));
    EXPECT_NEAR(glm::dot(hair.ranges()[0].root.rotation, hair.ranges()[0].root.rotation), 1.f, 1.e-6f);
    in.restPositions[1] = in.restPositions[0];
    EXPECT_FALSE(hair.initialize({in}));
    in.restPositions[1] = {0.f, -1.f, 0.f};
    in.restPositions[0] = {1.f, 0.f, 0.f};
    EXPECT_FALSE(hair.initialize({in}));
    in.restPositions[0] = {0.f, 0.f, 0.f}; in.inverseMass = -1.f;
    EXPECT_FALSE(hair.initialize({in}));
}

TEST(HairSolverTest, PreservesCurvedRestShapeWithoutForces) {
    HairStrands hair;
    auto gen = single(); gen.curvature = 0.6f;
    ASSERT_TRUE(generateHairBundle(hair, gen));
    const auto initial = hair.particles().positions;
    HairSolver solver;
    auto p = solver.params(); p.gravity = Vector3df(0.f);
    ASSERT_TRUE(solver.setParams(p));
    ASSERT_TRUE(solver.setStrands(&hair));
    for (int i = 0; i < 120; ++i) ASSERT_TRUE(solver.step());
    for (size_t i = 0; i < initial.size(); ++i)
        EXPECT_LT(glm::length(initial[i] - hair.particles().positions[i]), 2.e-5f);
    EXPECT_EQ(solver.stats().nonFiniteCount, 0u);
}

TEST(HairSolverTest, GravityMovesTipsKeepsRootsAndLengths) {
    HairStrands hair;
    ASSERT_TRUE(generateHairBundle(hair, single()));
    const auto initial = hair.particles().positions;
    HairSolver solver;
    ASSERT_TRUE(solver.setStrands(&hair));
    float worst = 0.f;
    for (int i = 0; i < 240; ++i) {
        ASSERT_TRUE(solver.step());
        worst = std::max(worst, solver.stats().maxRelativeLengthError);
    }
    EXPECT_FLOAT_EQ(glm::length(hair.particles().positions[0] - initial[0]), 0.f);
    EXPECT_GT(glm::length(hair.particles().positions.back() - initial.back()), 0.01f);
    EXPECT_LT(worst, 0.01f);
    EXPECT_EQ(solver.stats().steps, 240u);
    EXPECT_NEAR(solver.stats().simulatedTime, 4., 1.e-6);
}

TEST(HairSolverTest, ResetClearsMotionAndClock) {
    HairStrands hair;
    ASSERT_TRUE(generateHairBundle(hair, single()));
    const auto initial = hair.particles().positions;
    HairSolver solver;
    solver.setStrands(&hair);
    for (int i = 0; i < 30; ++i) ASSERT_TRUE(solver.step());
    solver.reset();
    for (size_t i = 0; i < hair.particleCount(); ++i) {
        EXPECT_FLOAT_EQ(glm::length(hair.particles().positions[i] - initial[i]), 0.f);
        EXPECT_FLOAT_EQ(glm::length(hair.particles().velocities[i]), 0.f);
    }
    EXPECT_EQ(solver.stats().steps, 0u);
    EXPECT_EQ(solver.stats().simulatedTime, 0.);
}

TEST(HairSolverTest, RejectsInvalidParametersAtomically) {
    HairSolver solver;
    const auto initial = solver.params();
    auto p = initial; p.numSubsteps = 0;
    EXPECT_FALSE(solver.setParams(p));
    p = initial; p.numIterations = 0;
    EXPECT_FALSE(solver.setParams(p));
    p = initial; p.timeStep = 0.f;
    EXPECT_FALSE(solver.setParams(p));
    p = initial; p.bendCompliance = -1.f;
    EXPECT_FALSE(solver.setParams(p));
    p = initial; p.gravity.x = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(solver.setParams(p));
    p = initial; p.dampingRate = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(solver.setParams(p));
    EXPECT_EQ(solver.params().timeStep, initial.timeStep);
    EXPECT_EQ(solver.params().numSubsteps, initial.numSubsteps);
}

TEST(HairSolverTest, EmptyDetachedAndReinitializedDataAreSafe) {
    HairSolver solver;
    EXPECT_FALSE(solver.step());
    HairStrands hair;
    solver.setStrands(&hair);
    EXPECT_FALSE(solver.step());
    ASSERT_TRUE(generateHairBundle(hair, single()));
    EXPECT_FALSE(solver.step()); // topology revision changed without a rebind
    solver.setStrands(&hair);
    EXPECT_TRUE(solver.step());
    solver.setStrands(nullptr);
    solver.reset();
    EXPECT_FALSE(solver.step());
}

TEST(HairSolverTest, ShapeComplianceControlsRestoration) {
    auto run = [](float shapeCompliance) {
        HairStrands hair;
        generateHairBundle(hair, single());
        const auto tip = hair.particles().positions.back();
        HairSolver solver;
        auto p = solver.params(); p.shapeCompliance = shapeCompliance;
        solver.setParams(p); solver.setStrands(&hair);
        for (int i = 0; i < 120; ++i) solver.step();
        return glm::length(tip - hair.particles().positions.back());
    };
    EXPECT_GT(run(0.1f), run(0.f) + 0.01f);
}

TEST(HairSolverTest, TimeStepAndSubstepsRemainStable) {
    std::vector<Vector3df> tips;
    for (float dt : {1.f/30.f, 1.f/60.f, 1.f/120.f}) {
        HairStrands hair;
        generateHairBundle(hair, single());
        HairSolver solver;
        auto p = solver.params(); p.timeStep = dt;
        p.numSubsteps = static_cast<int>(std::round(dt*480.f));
        solver.setParams(p); solver.setStrands(&hair);
        for (int i = 0; i < static_cast<int>(std::round(1.f/dt)); ++i) ASSERT_TRUE(solver.step());
        tips.push_back(hair.particles().positions.back());
        EXPECT_LT(solver.stats().maxRelativeLengthError, 0.01f);
    }
    EXPECT_LT(glm::length(tips[0]-tips[1]), 2.e-4f);
    EXPECT_LT(glm::length(tips[1]-tips[2]), 2.e-4f);
}

TEST(HairSolverTest, DefaultDisplayBundleRemainsWithinLengthBudget) {
    SKIP_IN_DEBUG_SLOW();
    HairStrands hair;
    ASSERT_TRUE(generateHairBundle(hair, HairGeneratorParams{}));
    HairSolver solver;
    solver.setStrands(&hair);
    float worst = 0.f;
    for (int i = 0; i < 120; ++i) {
        ASSERT_TRUE(solver.step());
        worst = std::max(worst, solver.stats().maxRelativeLengthError);
    }
    EXPECT_LT(worst, 0.01f);
    for (const auto& r : hair.ranges())
        EXPECT_FLOAT_EQ(glm::length(hair.particles().positions[r.offset] - r.root.position), 0.f);
}
