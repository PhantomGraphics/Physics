#pragma once

#include <string>

namespace Phantom {

// Shared wire / shaded / both toggle for the rigid- and soft-body PBR passes
// (GltfBodyRenderer / GltfSoftRenderer) -- see
// docs/todo/PLAN_physicsview_gltf_rendering.md Phase 2/3. The wireframe renderer
// (RigidBodyWireRenderer / SoftBodyWireRenderer) and the shaded renderer are
// separate sub-renderers; FluidApp reads the mode to gate each one's
// setEnabled().
enum class BodyRenderMode { Wireframe, Shaded, Both };

inline bool parseBodyRenderMode(const std::string& s, BodyRenderMode& out) {
    if (s == "wire" || s == "wireframe") { out = BodyRenderMode::Wireframe; return true; }
    if (s == "shaded")                   { out = BodyRenderMode::Shaded;    return true; }
    if (s == "both")                     { out = BodyRenderMode::Both;      return true; }
    return false;
}

inline const char* bodyRenderModeName(BodyRenderMode m) {
    switch (m) {
    case BodyRenderMode::Wireframe: return "wire";
    case BodyRenderMode::Shaded:    return "shaded";
    case BodyRenderMode::Both:      return "both";
    }
    return "wire";
}

inline bool bodyRenderModeWantsWire(BodyRenderMode m) {
    return m == BodyRenderMode::Wireframe || m == BodyRenderMode::Both;
}
inline bool bodyRenderModeWantsShaded(BodyRenderMode m) {
    return m == BodyRenderMode::Shaded || m == BodyRenderMode::Both;
}

} // namespace Phantom
