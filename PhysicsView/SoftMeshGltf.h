#pragma once

#include "../../CGLib/GltfRenderer/Gltf/GltfDocument.h"
#include "Physics/Physics/SoftMesh.h"

#include <glm/glm.hpp>

#include <vector>

namespace Phantom {

// Vulkan-independent synthesis for GltfSoftRenderer
// (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 3).
//
// The document has one primitive: one vertex per soft particle (indexed /
// shared, so normals are smooth), triangles wound as authored. A cloth is a
// zero-thickness surface, so GltfSoftRenderer renders it with the culling
// disabled (GltfSceneRenderer::setCullMode(VK_CULL_MODE_NONE)) -- double-winding
// instead (§2.3's other option) makes coincident front/back triangles that
// z-fight into a moire. PhysicsView's gltf.frag flips the normal toward the
// viewer (dot(N,V) < 0), so the back of the cloth is lit correctly too.
//
// Positions + normals are re-streamed every frame through
// GltfSceneRenderer::updateMorphedGeometry(); the vertex count therefore stays
// == positions.size() and the primitive is marked dynamic.

// Build the static document (single winding, as authored); the initial normals
// are computeSoftBodyNormals(positions, faces).
Phantom::Gltf::GltfDocument makeSoftBodyGltf(const std::vector<glm::vec3>& positions,
                                            const std::vector<Physics::SoftFace>& faces,
                                            const glm::vec4& baseColor);

// Per-frame smooth per-vertex normals for `positions` (area-weighted
// accumulation of authored-winding face normals, then normalized).
// outNormals is resized to positions.size().
void computeSoftBodyNormals(const std::vector<glm::vec3>& positions,
                            const std::vector<Physics::SoftFace>& faces,
                            std::vector<glm::vec3>& outNormals);

} // namespace Phantom
