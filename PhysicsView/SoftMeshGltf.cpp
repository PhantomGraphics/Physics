#include "SoftMeshGltf.h"

#include "../../CGLib/GltfRenderer/Gltf/GltfAccessorBuilder.h"

#include <cstdint>

namespace Phantom {

using Phantom::Gltf::GltfDocument;

void computeSoftBodyNormals(const std::vector<glm::vec3>& positions,
                            const std::vector<Physics::SoftFace>& faces,
                            std::vector<glm::vec3>& outNormals) {
    outNormals.assign(positions.size(), glm::vec3(0.f));
    const int n = static_cast<int>(positions.size());
    for (const auto& f : faces) {
        if (f.a < 0 || f.b < 0 || f.c < 0 || f.a >= n || f.b >= n || f.c >= n) continue;
        // Area-weighted (unnormalized cross) so larger triangles contribute more.
        const glm::vec3 fn = glm::cross(positions[f.b] - positions[f.a],
                                        positions[f.c] - positions[f.a]);
        outNormals[f.a] += fn;
        outNormals[f.b] += fn;
        outNormals[f.c] += fn;
    }
    for (auto& nrm : outNormals) {
        const float len = glm::length(nrm);
        nrm = (len > 1e-8f) ? (nrm / len) : glm::vec3(0.f, 1.f, 0.f);
    }
}

GltfDocument makeSoftBodyGltf(const std::vector<glm::vec3>& positions,
                              const std::vector<Physics::SoftFace>& faces,
                              const glm::vec4& baseColor) {
    using namespace Phantom::Gltf;

    std::vector<glm::vec3> normals;
    computeSoftBodyNormals(positions, faces, normals);

    const int n = static_cast<int>(positions.size());
    std::vector<uint32_t> indices;
    indices.reserve(faces.size() * 3);
    for (const auto& f : faces) {
        if (f.a < 0 || f.b < 0 || f.c < 0 || f.a >= n || f.b >= n || f.c >= n) continue;
        indices.insert(indices.end(), { static_cast<uint32_t>(f.a),
                                        static_cast<uint32_t>(f.b),
                                        static_cast<uint32_t>(f.c) });
    }

    GltfDocument doc;

    GltfMaterial mat;
    mat.name = "softbody";
    mat.pbrMetallicRoughness.baseColorFactor = baseColor;
    mat.pbrMetallicRoughness.metallicFactor  = 0.0f;
    mat.pbrMetallicRoughness.roughnessFactor = 0.6f;
    mat.doubleSided = true;
    doc.materials.push_back(std::move(mat));

    GltfPrimitive prim;
    prim.positionAccessor = appendAccessor(doc, positions, GltfComponentType::Float,       GltfAccessorType::Vec3);
    prim.normalAccessor   = appendAccessor(doc, normals,   GltfComponentType::Float,       GltfAccessorType::Vec3);
    prim.indicesAccessor  = appendAccessor(doc, indices,   GltfComponentType::UnsignedInt, GltfAccessorType::Scalar);
    prim.materialIndex    = 0;

    GltfMesh mesh;
    mesh.name = "softbody";
    mesh.primitives.push_back(prim);
    doc.meshes.push_back(std::move(mesh));

    GltfNode node;
    node.name      = "softbody";
    node.meshIndex = 0;
    doc.nodes.push_back(std::move(node));

    GltfScene scene;
    scene.nodes.push_back(0);
    doc.scenes.push_back(std::move(scene));
    doc.defaultScene = 0;

    return doc;
}

} // namespace Phantom
