// Unit tests for PhysicsView/SoftMeshGltf.{h,cpp} -- the pure-logic
// (Vulkan-independent) synthesis of a soft-body surface GltfDocument used by
// GltfSoftRenderer (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 3 / §6).
// SoftMeshGltf.cpp is compiled straight into this test executable (see
// Physics/CMakeLists.txt).

#include <gtest/gtest.h>

#include "../PhysicsView/SoftMeshGltf.h"

#include <cmath>
#include <cstring>
#include <vector>

using Phantom::Gltf::GltfDocument;
using Phantom::Physics::SoftFace;

namespace {

std::vector<glm::vec3> readVec3(const GltfDocument& doc, int accessorIdx) {
    const auto& acc = doc.accessors[accessorIdx];
    const auto& view = doc.bufferViews[acc.bufferViewIndex];
    const auto& buf = doc.buffers[view.bufferIndex];
    std::vector<glm::vec3> out(acc.count);
    std::memcpy(out.data(), buf.data.data() + view.byteOffset, acc.count * sizeof(glm::vec3));
    return out;
}

std::vector<uint32_t> readIndices(const GltfDocument& doc, int accessorIdx) {
    const auto& acc = doc.accessors[accessorIdx];
    const auto& view = doc.bufferViews[acc.bufferViewIndex];
    const auto& buf = doc.buffers[view.bufferIndex];
    std::vector<uint32_t> out(acc.count);
    std::memcpy(out.data(), buf.data.data() + view.byteOffset, acc.count * sizeof(uint32_t));
    return out;
}

// A unit square in the y=0 plane: p0..p3 CCW, two triangles.
std::vector<glm::vec3> quadPositions() {
    return { {0,0,0}, {1,0,0}, {1,0,1}, {0,0,1} };
}
std::vector<SoftFace> quadFaces() {
    return { {0, 1, 2}, {0, 2, 3} };
}

} // namespace

TEST(SoftMeshGltf, SharedVertexTopology) {
    GltfDocument doc = Phantom::makeSoftBodyGltf(quadPositions(), quadFaces(),
                                                glm::vec4(0.8f, 0.4f, 0.5f, 1.0f));

    ASSERT_EQ(doc.meshes.size(), 1u);
    ASSERT_EQ(doc.materials.size(), 1u);
    EXPECT_TRUE(doc.materials[0].doubleSided);

    const auto& prim = doc.meshes[0].primitives[0];
    ASSERT_GE(prim.positionAccessor, 0);
    ASSERT_GE(prim.normalAccessor, 0);
    ASSERT_GE(prim.indicesAccessor, 0);

    // one vertex per particle (shared -> smooth normals)
    EXPECT_EQ(doc.accessors[prim.positionAccessor].count, 4u);
    EXPECT_EQ(doc.accessors[prim.normalAccessor].count, 4u);
    // 2 faces x 3 indices -- single winding (GltfSoftRenderer renders cull-none)
    EXPECT_EQ(doc.accessors[prim.indicesAccessor].count, 6u);

    const auto idx = readIndices(doc, prim.indicesAccessor);
    EXPECT_EQ(idx[0], 0u); EXPECT_EQ(idx[1], 1u); EXPECT_EQ(idx[2], 2u);
    EXPECT_EQ(idx[3], 0u); EXPECT_EQ(idx[4], 2u); EXPECT_EQ(idx[5], 3u);

    const auto pos = readVec3(doc, prim.positionAccessor);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(pos[i].x, quadPositions()[i].x);
        EXPECT_FLOAT_EQ(pos[i].z, quadPositions()[i].z);
    }
}

TEST(SoftMeshGltf, NormalsFollowAuthoredWinding) {
    std::vector<glm::vec3> normals;
    Phantom::computeSoftBodyNormals(quadPositions(), quadFaces(), normals);

    ASSERT_EQ(normals.size(), 4u);
    for (const auto& n : normals) {
        EXPECT_NEAR(glm::length(n), 1.0f, 1e-5f);
        // faces {0,1,2}/{0,2,3} wound as authored give a -Y geometric normal
        EXPECT_NEAR(n.x, 0.0f, 1e-5f);
        EXPECT_NEAR(n.y, -1.0f, 1e-5f);
        EXPECT_NEAR(n.z, 0.0f, 1e-5f);
    }
}

TEST(SoftMeshGltf, NormalsStayUnitAfterDeformation) {
    // Fold the quad along its diagonal.
    std::vector<glm::vec3> deformed = { {0,0,0}, {1,0.5f,0}, {1,0,1}, {0,0.5f,1} };
    std::vector<glm::vec3> normals;
    Phantom::computeSoftBodyNormals(deformed, quadFaces(), normals);

    ASSERT_EQ(normals.size(), 4u);
    for (const auto& n : normals)
        EXPECT_NEAR(glm::length(n), 1.0f, 1e-5f);
}

TEST(SoftMeshGltf, DegenerateFaceIndicesAreSkipped) {
    std::vector<SoftFace> faces = { {0, 1, 2}, {0, 2, 9 /* out of range */} };
    GltfDocument doc = Phantom::makeSoftBodyGltf(quadPositions(), faces, glm::vec4(1.0f));
    // only the valid face survives -> 1 x 3
    EXPECT_EQ(doc.accessors[doc.meshes[0].primitives[0].indicesAccessor].count, 3u);
}
