#pragma once

#include "../../CGLib/GltfRenderer/Gltf/GltfDocument.h"

#include <glm/glm.hpp>

namespace Phantom {

// Pure-logic (no Vulkan) synthesis of unit collision-shape meshes as
// single-primitive GltfDocuments -- the per-body geometry GltfBodyRenderer
// instances (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 2).
//
// Each returns a document with one PBR material (the given linear base colour,
// metallic 0 / roughness 0.55) and one mesh/primitive carrying POSITION +
// NORMAL + indexed triangles wound CCW so GltfSceneRenderer's
// VK_CULL_MODE_BACK_BIT + VK_FRONT_FACE_COUNTER_CLOCKWISE pipeline shows them
// solid. The caller scales/orients the whole document per frame via
// GltfSceneRenderer::setModelMatrix().

// Unit sphere, radius 1, centred at the origin. latSegments rings between the
// poles, lonSegments slices around; the defaults (16 x 24) are ~700 triangles.
Phantom::Gltf::GltfDocument makeUnitSphereGltf(const glm::vec4& baseColor,
                                              int latSegments = 16,
                                              int lonSegments = 24);

// Unit box spanning [-1,1] on every axis (i.e. half-extents 1), per-face flat
// normals.
Phantom::Gltf::GltfDocument makeUnitBoxGltf(const glm::vec4& baseColor);

// A flat square in the y=0 plane, +Y normal, spanning [-halfSize, halfSize] in
// x and z -- a finite stand-in for an infinite PlaneShape floor.
Phantom::Gltf::GltfDocument makePlaneGltf(const glm::vec4& baseColor,
                                          float halfSize = 40.0f);

} // namespace Phantom
