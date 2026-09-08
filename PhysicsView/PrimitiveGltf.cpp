#include "PrimitiveGltf.h"

#include "../../CGLib/GltfRenderer/Gltf/GltfAccessorBuilder.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace Phantom {

using Phantom::Gltf::GltfDocument;

namespace {

constexpr float kPi = 3.14159265358979323846f;

GltfDocument finishDocument(std::vector<glm::vec3>& positions,
                            std::vector<glm::vec3>& normals,
                            std::vector<uint32_t>& indices,
                            const glm::vec4& baseColor) {
    using namespace Phantom::Gltf;
    GltfDocument doc;

    GltfMaterial mat;
    mat.name = "primitive";
    mat.pbrMetallicRoughness.baseColorFactor = baseColor;
    mat.pbrMetallicRoughness.metallicFactor  = 0.0f;
    mat.pbrMetallicRoughness.roughnessFactor = 0.55f;
    doc.materials.push_back(std::move(mat));

    GltfPrimitive prim;
    prim.positionAccessor = appendAccessor(doc, positions, GltfComponentType::Float,       GltfAccessorType::Vec3);
    prim.normalAccessor   = appendAccessor(doc, normals,   GltfComponentType::Float,       GltfAccessorType::Vec3);
    prim.indicesAccessor  = appendAccessor(doc, indices,   GltfComponentType::UnsignedInt, GltfAccessorType::Scalar);
    prim.materialIndex    = 0;

    GltfMesh mesh;
    mesh.name = "primitive";
    mesh.primitives.push_back(prim);
    doc.meshes.push_back(std::move(mesh));

    GltfNode node;
    node.name      = "primitive";
    node.meshIndex = 0;
    doc.nodes.push_back(std::move(node));

    GltfScene scene;
    scene.nodes.push_back(0);
    doc.scenes.push_back(std::move(scene));
    doc.defaultScene = 0;

    return doc;
}

// One quad, corners listed CCW when viewed from the +normal side.
void addQuad(std::vector<glm::vec3>& P, std::vector<glm::vec3>& N, std::vector<uint32_t>& I,
             const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
             const glm::vec3& n) {
    const uint32_t base = static_cast<uint32_t>(P.size());
    for (const auto& v : { a, b, c, d }) { P.push_back(v); N.push_back(n); }
    I.insert(I.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
}

} // namespace

GltfDocument makeUnitSphereGltf(const glm::vec4& baseColor, int latSegments, int lonSegments) {
    if (latSegments < 2) latSegments = 2;
    if (lonSegments < 3) lonSegments = 3;

    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<uint32_t>  indices;

    // (latSegments+1) x (lonSegments+1) grid of vertices (seam duplicated so the
    // UV-less normals stay per-vertex correct at the wrap).
    for (int lat = 0; lat <= latSegments; ++lat) {
        const float theta = kPi * static_cast<float>(lat) / static_cast<float>(latSegments);
        const float st = std::sin(theta);
        const float ct = std::cos(theta);
        for (int lon = 0; lon <= lonSegments; ++lon) {
            const float phi = 2.0f * kPi * static_cast<float>(lon) / static_cast<float>(lonSegments);
            const glm::vec3 p(st * std::cos(phi), ct, st * std::sin(phi));
            positions.push_back(p);
            normals.push_back(p); // unit sphere: position == outward normal
        }
    }

    const int stride = lonSegments + 1;
    for (int lat = 0; lat < latSegments; ++lat) {
        for (int lon = 0; lon < lonSegments; ++lon) {
            const uint32_t a = static_cast<uint32_t>(lat * stride + lon);
            const uint32_t b = a + 1;
            const uint32_t c = a + stride;
            const uint32_t d = c + 1;
            // (a,b,c) and (b,d,c) are CCW when seen from outside the sphere.
            indices.insert(indices.end(), { a, b, c, b, d, c });
        }
    }

    return finishDocument(positions, normals, indices, baseColor);
}

GltfDocument makeUnitBoxGltf(const glm::vec4& baseColor) {
    std::vector<glm::vec3> P;
    std::vector<glm::vec3> N;
    std::vector<uint32_t>  I;
    const float h = 1.0f;
    // Corners listed CCW from the outside; matches gen_render_assets.py's box().
    addQuad(P, N, I, {-h,-h, h}, { h,-h, h}, { h, h, h}, {-h, h, h}, { 0, 0, 1}); // +Z
    addQuad(P, N, I, { h,-h,-h}, {-h,-h,-h}, {-h, h,-h}, { h, h,-h}, { 0, 0,-1}); // -Z
    addQuad(P, N, I, { h,-h, h}, { h,-h,-h}, { h, h,-h}, { h, h, h}, { 1, 0, 0}); // +X
    addQuad(P, N, I, {-h,-h,-h}, {-h,-h, h}, {-h, h, h}, {-h, h,-h}, {-1, 0, 0}); // -X
    addQuad(P, N, I, {-h, h, h}, { h, h, h}, { h, h,-h}, {-h, h,-h}, { 0, 1, 0}); // +Y
    addQuad(P, N, I, {-h,-h,-h}, { h,-h,-h}, { h,-h, h}, {-h,-h, h}, { 0,-1, 0}); // -Y
    return finishDocument(P, N, I, baseColor);
}

GltfDocument makePlaneGltf(const glm::vec4& baseColor, float halfSize) {
    std::vector<glm::vec3> P;
    std::vector<glm::vec3> N;
    std::vector<uint32_t>  I;
    const float s = halfSize;
    addQuad(P, N, I, {-s, 0.f, s}, { s, 0.f, s}, { s, 0.f,-s}, {-s, 0.f,-s}, { 0.f, 1.f, 0.f });
    return finishDocument(P, N, I, baseColor);
}

} // namespace Phantom
