#pragma once

#include <functional>
#include <string>
#include <vector>

namespace Phantom {

/** @brief Category of a scene object tracked by SceneComponentRegistry. */
enum class SceneComponentKind {
    Fluid,
    RigidBody,
    SoftBody,
    MeshBoundary,
    Emitter,
    OutflowRegion,
};

/**
 * @brief A View-side handle for one object in the PhysicsView scene.
 *
 * PhysicsView keeps its objects in three separate worlds (FluidWorld / its
 * RigidBodyWorld / SoftBodyWorld) that don't track identity. Each world
 * registers a SceneComponent per object it owns; the component carries a
 * stable id (kept for the object's lifetime) plus a describe() callback the
 * object-list panel evaluates on demand for a live one-line detail.
 */
struct SceneComponent {
    int                          id = 0;
    SceneComponentKind           kind = SceneComponentKind::Fluid;
    std::string                  label;     ///< category, e.g. "RigidBody"
    std::function<std::string()> describe;  ///< live one-line detail
};

/**
 * @brief Owns the flat list of every SceneComponent in the scene and hands
 * out monotonically increasing ids. FluidApp owns one; each world registers
 * into it. Registration order is preserved, so the panel can just walk the
 * list.
 */
class SceneComponentRegistry {
public:
    int  add(SceneComponentKind kind, std::string label,
             std::function<std::string()> describe);
    void remove(int id);

    const std::vector<SceneComponent>& components() const { return components_; }
    int count(SceneComponentKind kind) const;

    static const char* kindName(SceneComponentKind kind);

private:
    std::vector<SceneComponent> components_;
    int nextId_ = 1;
};

} // namespace Phantom
