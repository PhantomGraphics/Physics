#include "pch.h"
#include "../Physics/MeshBoundaryShape.h"
#include "../Physics/ICollisionShape.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);

// Voxelized SDF accuracy is bounded by voxelSize, not float epsilon.
constexpr float kSdfTol    = 0.06f;
constexpr float kNormalTol = 0.25f;

// Builds a closed unit box (half-extents 0.5 on every axis, centered at the
// origin) as 12 triangles -- winding doesn't matter for LevelSet's ray-parity
// inside/outside test.
std::vector<Triangle3df> buildUnitBoxTriangles(const Vector3df& h) {
    const Vector3df c[8] = {
        { -h.x, -h.y, -h.z }, {  h.x, -h.y, -h.z }, {  h.x,  h.y, -h.z }, { -h.x,  h.y, -h.z },
        { -h.x, -h.y,  h.z }, {  h.x, -h.y,  h.z }, {  h.x,  h.y,  h.z }, { -h.x,  h.y,  h.z },
    };

    auto tri = [](const Vector3df& a, const Vector3df& b, const Vector3df& cc) {
        return Triangle3df(std::array<Vector3df, 3>{ a, b, cc });
    };

    return {
        tri(c[0], c[1], c[2]), tri(c[0], c[2], c[3]),  // -Z
        tri(c[4], c[6], c[5]), tri(c[4], c[7], c[6]),  // +Z
        tri(c[0], c[5], c[1]), tri(c[0], c[4], c[5]),  // -Y
        tri(c[3], c[2], c[6]), tri(c[3], c[6], c[7]),  // +Y
        tri(c[0], c[3], c[7]), tri(c[0], c[7], c[4]),  // -X
        tri(c[1], c[5], c[6]), tri(c[1], c[6], c[2]),  // +X
    };
}
}

TEST(MeshBoundaryShapeTest, GetType_IsMesh) {
    const Vector3df h(0.5f, 0.5f, 0.5f);
    MeshBoundaryShape mesh(buildUnitBoxTriangles(h), 0.1f);
    EXPECT_EQ(mesh.getType(), ShapeType::Mesh);
    EXPECT_EQ(mesh.getTriangleCount(), 12u);
}

TEST(MeshBoundaryShapeTest, SignedDistance_MatchesBoxShape_AtCenterFaceAndOutside) {
    const Vector3df h(0.5f, 0.5f, 0.5f);
    MeshBoundaryShape mesh(buildUnitBoxTriangles(h), 0.1f);

    BoxShape refBox;
    refBox.halfExtents = h;

    const Vector3df pos(0.f, 0.f, 0.f);

    // Note: the SDF is only actively computed within ~1 voxel of the mesh's
    // AABB (see LevelSet::setSignedDistance()'s padding) -- queries further out
    // than that legitimately fall back to the background sentinel, so samples
    // here stay within that padded band.
    const std::vector<Vector3df> samples = {
        { 0.f, 0.f, 0.f },     // center: -0.5
        { 0.5f, 0.f, 0.f },    // on +X face: 0.0
        { 0.58f, 0.f, 0.f },   // just outside, within the padded band: 0.08
        { 0.3f, 0.2f, -0.1f }, // interior, off-axis
    };

    for (const auto& p : samples) {
        const float expected = refBox.getSignedDistance(p, pos, kIdentity);
        const float actual   = mesh.getSignedDistance(p, pos, kIdentity);
        EXPECT_NEAR(actual, expected, kSdfTol) << "at (" << p.x << "," << p.y << "," << p.z << ")";
    }
}

TEST(MeshBoundaryShapeTest, SurfaceNormal_PointsOutwardAtFaceCenters) {
    const Vector3df h(0.5f, 0.5f, 0.5f);
    MeshBoundaryShape mesh(buildUnitBoxTriangles(h), 0.1f);
    const Vector3df pos(0.f, 0.f, 0.f);

    const std::vector<std::pair<Vector3df, Vector3df>> faces = {
        { { 0.6f, 0.f, 0.f }, { 1.f, 0.f, 0.f } },
        { { -0.6f, 0.f, 0.f }, { -1.f, 0.f, 0.f } },
        { { 0.f, 0.6f, 0.f }, { 0.f, 1.f, 0.f } },
        { { 0.f, -0.6f, 0.f }, { 0.f, -1.f, 0.f } },
        { { 0.f, 0.f, 0.6f }, { 0.f, 0.f, 1.f } },
        { { 0.f, 0.f, -0.6f }, { 0.f, 0.f, -1.f } },
    };

    for (const auto& [p, expectedDir] : faces) {
        const Vector3df n = mesh.getSurfaceNormal(p, pos, kIdentity);
        EXPECT_NEAR(getLength(n), 1.f, 0.1f);
        EXPECT_GT(glm::dot(n, expectedDir), 1.f - kNormalTol);
    }
}

TEST(MeshBoundaryShapeTest, GetAABB_MatchesBoxHalfExtents) {
    const Vector3df h(0.5f, 1.f, 2.f);
    MeshBoundaryShape mesh(buildUnitBoxTriangles(h), 0.1f);

    const Vector3df pos(1.f, 2.f, 3.f);
    Box3df box = mesh.getAABB(pos, kIdentity);

    EXPECT_NEAR(box.getMin().x, pos.x - h.x, 1.0e-4f);
    EXPECT_NEAR(box.getMin().y, pos.y - h.y, 1.0e-4f);
    EXPECT_NEAR(box.getMin().z, pos.z - h.z, 1.0e-4f);
    EXPECT_NEAR(box.getMax().x, pos.x + h.x, 1.0e-4f);
    EXPECT_NEAR(box.getMax().y, pos.y + h.y, 1.0e-4f);
    EXPECT_NEAR(box.getMax().z, pos.z + h.z, 1.0e-4f);
}

TEST(MeshBoundaryShapeTest, FarOutsideQuery_LargePositiveDistance_NoBoundaryForce) {
    const Vector3df h(0.5f, 0.5f, 0.5f);
    MeshBoundaryShape mesh(buildUnitBoxTriangles(h), 0.1f);

    // Far outside the mesh's padded AABB: falls back to the background sentinel.
    const float d = mesh.getSignedDistance({ 1000.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, kIdentity);
    EXPECT_GT(d, 100.f);
}
