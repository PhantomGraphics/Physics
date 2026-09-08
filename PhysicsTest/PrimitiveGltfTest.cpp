// Unit tests for PhysicsView/PrimitiveGltf.{h,cpp} -- the pure-logic
// (Vulkan-independent) synthesis of unit collision-shape meshes used by
// GltfBodyRenderer (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 2 / §6).
//
// PrimitiveGltf.cpp is compiled straight into this test executable (see
// Physics/CMakeLists.txt) -- it only needs GltfRenderer's header-only
// GltfDocument/GltfAccessorBuilder plus glm, no GltfRendererCore link.

#include <gtest/gtest.h>

#include "../PhysicsView/PrimitiveGltf.h"

#include <cmath>
#include <cstring>
#include <vector>

using Phantom::Gltf::GltfDocument;
using Phantom::Gltf::GltfComponentType;

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

// The winding contract: for every triangle, the geometric normal derived from
// vertex order (cross of two edges) must point the same way as the stored
// per-vertex normals -- i.e. CCW as seen from outside, so GltfSceneRenderer's
// back-face cull shows the primitive solid.
void expectOutwardWinding(const GltfDocument& doc) {
    const auto& prim = doc.meshes[0].primitives[0];
    const auto pos = readVec3(doc, prim.positionAccessor);
    const auto nrm = readVec3(doc, prim.normalAccessor);
    const auto idx = readIndices(doc, prim.indicesAccessor);
    ASSERT_EQ(idx.size() % 3u, 0u);

    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        const glm::vec3& a = pos[idx[t]];
        const glm::vec3& b = pos[idx[t + 1]];
        const glm::vec3& c = pos[idx[t + 2]];
        const glm::vec3 geo = glm::cross(b - a, c - a);
        // Degenerate (zero-area) triangles -- e.g. the collapsed pole row of a
        // UV sphere -- have no winding; they rasterize nothing, so skip them.
        if (glm::length(geo) < 1e-5f) continue;
        const glm::vec3 avgN = nrm[idx[t]] + nrm[idx[t + 1]] + nrm[idx[t + 2]];
        EXPECT_GT(glm::dot(geo, avgN), 0.0f)
            << "triangle " << (t / 3) << " is wound clockwise (inward)";
    }
}

} // namespace

TEST(PrimitiveGltf, UnitBoxTopology) {
    const glm::vec4 color(0.2f, 0.4f, 0.6f, 1.0f);
    GltfDocument doc = Phantom::makeUnitBoxGltf(color);

    ASSERT_EQ(doc.meshes.size(), 1u);
    ASSERT_EQ(doc.meshes[0].primitives.size(), 1u);
    ASSERT_EQ(doc.materials.size(), 1u);
    ASSERT_EQ(doc.scenes.size(), 1u);
    ASSERT_EQ(doc.nodes.size(), 1u);

    const auto& prim = doc.meshes[0].primitives[0];
    EXPECT_EQ(prim.materialIndex, 0);
    ASSERT_GE(prim.positionAccessor, 0);
    ASSERT_GE(prim.normalAccessor, 0);
    ASSERT_GE(prim.indicesAccessor, 0);

    EXPECT_EQ(doc.accessors[prim.positionAccessor].count, 24u); // 6 faces x 4
    EXPECT_EQ(doc.accessors[prim.indicesAccessor].count, 36u);  // 6 faces x 2 tris x 3
    EXPECT_EQ(doc.accessors[prim.indicesAccessor].componentType, GltfComponentType::UnsignedInt);

    const auto pos = readVec3(doc, prim.positionAccessor);
    for (const auto& p : pos) {
        EXPECT_NEAR(std::fabs(p.x), 1.0f, 1e-5f);
        EXPECT_NEAR(std::fabs(p.y), 1.0f, 1e-5f);
        EXPECT_NEAR(std::fabs(p.z), 1.0f, 1e-5f);
    }

    const auto& bc = doc.materials[0].pbrMetallicRoughness.baseColorFactor;
    EXPECT_FLOAT_EQ(bc.r, color.r);
    EXPECT_FLOAT_EQ(bc.a, color.a);
    EXPECT_FLOAT_EQ(doc.materials[0].pbrMetallicRoughness.metallicFactor, 0.0f);
}

TEST(PrimitiveGltf, UnitBoxWinding) {
    expectOutwardWinding(Phantom::makeUnitBoxGltf(glm::vec4(1.0f)));
}

TEST(PrimitiveGltf, UnitSphereTopologyAndWinding) {
    const int lat = 4, lon = 6;
    GltfDocument doc = Phantom::makeUnitSphereGltf(glm::vec4(1.0f), lat, lon);
    const auto& prim = doc.meshes[0].primitives[0];

    EXPECT_EQ(doc.accessors[prim.positionAccessor].count,
              static_cast<size_t>((lat + 1) * (lon + 1)));
    EXPECT_EQ(doc.accessors[prim.indicesAccessor].count,
              static_cast<size_t>(lat * lon * 6));

    const auto pos = readVec3(doc, prim.positionAccessor);
    const auto nrm = readVec3(doc, prim.normalAccessor);
    for (size_t i = 0; i < pos.size(); ++i) {
        EXPECT_NEAR(glm::length(pos[i]), 1.0f, 1e-5f);           // on the unit sphere
        EXPECT_NEAR(glm::length(nrm[i] - pos[i]), 0.0f, 1e-5f);  // normal == position
    }
    expectOutwardWinding(doc);
}

TEST(PrimitiveGltf, PlaneIsFlatAndUpward) {
    GltfDocument doc = Phantom::makePlaneGltf(glm::vec4(1.0f), 10.0f);
    const auto& prim = doc.meshes[0].primitives[0];

    EXPECT_EQ(doc.accessors[prim.positionAccessor].count, 4u);
    EXPECT_EQ(doc.accessors[prim.indicesAccessor].count, 6u);

    const auto pos = readVec3(doc, prim.positionAccessor);
    const auto nrm = readVec3(doc, prim.normalAccessor);
    for (const auto& p : pos) {
        EXPECT_FLOAT_EQ(p.y, 0.0f);
        EXPECT_LE(std::fabs(p.x), 10.0f + 1e-5f);
        EXPECT_LE(std::fabs(p.z), 10.0f + 1e-5f);
    }
    for (const auto& n : nrm) {
        EXPECT_NEAR(n.x, 0.0f, 1e-6f);
        EXPECT_NEAR(n.y, 1.0f, 1e-6f);
        EXPECT_NEAR(n.z, 0.0f, 1e-6f);
    }
    expectOutwardWinding(doc);
}
