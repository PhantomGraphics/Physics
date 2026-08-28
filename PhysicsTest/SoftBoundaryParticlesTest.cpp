#include "pch.h"
#include "../Physics/SoftBoundaryParticles.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;

SoftMesh makeGridMesh() {
    SoftMesh mesh;
    mesh.generateGrid(3, 3, 1.f, 1.f);
    return mesh;
}
}

TEST(SoftBoundaryParticlesTest, Bind_CreatesOneEntryPerMeshParticle) {
    SoftMesh mesh = makeGridMesh();

    SoftBoundaryParticles sb;
    sb.bind(&mesh);

    EXPECT_EQ(sb.particles().size(), mesh.particles.size());
}

TEST(SoftBoundaryParticlesTest, Sync_CopiesWorldPositionsFromMesh) {
    SoftMesh mesh = makeGridMesh();
    mesh.particles.positions[0] = Vector3df(1.f, 2.f, 3.f);

    SoftBoundaryParticles sb;
    sb.bind(&mesh);
    sb.sync();

    EXPECT_NEAR(getDistance(sb.particles()[0].worldPos, mesh.particles.positions[0]), 0.f, kTol);
}

TEST(SoftBoundaryParticlesTest, ComputePsi_ProducesPositivePsiForNonDegenerateMesh) {
    SoftMesh mesh = makeGridMesh();

    SoftBoundaryParticles sb;
    sb.bind(&mesh);
    sb.sync();

    SPHKernel kernel(0.5f);
    sb.computePsi(kernel, 1000.f);

    ASSERT_FALSE(sb.particles().empty());
    for (auto& bp : sb.particles()) {
        EXPECT_GT(bp.psi, 0.f);
    }
}

TEST(SoftBoundaryParticlesTest, ClearAccumForce_ZeroesAllEntries) {
    SoftMesh mesh = makeGridMesh();

    SoftBoundaryParticles sb;
    sb.bind(&mesh);
    for (auto& bp : sb.particles()) bp.accumForce = Vector3df(1.f, 2.f, 3.f);

    sb.clearAccumForce();

    for (auto& bp : sb.particles()) {
        EXPECT_NEAR(getLength(bp.accumForce), 0.f, kTol);
    }
}
