#include "pch.h"
#include "SceneComponent.h"

#include <algorithm>

namespace Phantom {

int SceneComponentRegistry::add(SceneComponentKind kind, std::string label,
                                std::function<std::string()> describe)
{
    const int id = nextId_++;
    components_.push_back({ id, kind, std::move(label), std::move(describe) });
    return id;
}

void SceneComponentRegistry::remove(int id)
{
    components_.erase(
        std::remove_if(components_.begin(), components_.end(),
                       [id](const SceneComponent& c) { return c.id == id; }),
        components_.end());
}

int SceneComponentRegistry::count(SceneComponentKind kind) const
{
    return static_cast<int>(std::count_if(components_.begin(), components_.end(),
        [kind](const SceneComponent& c) { return c.kind == kind; }));
}

const char* SceneComponentRegistry::kindName(SceneComponentKind kind)
{
    switch (kind) {
    case SceneComponentKind::Fluid:         return "Fluid";
    case SceneComponentKind::RigidBody:     return "RigidBody";
    case SceneComponentKind::SoftBody:      return "SoftBody";
    case SceneComponentKind::MeshBoundary:  return "MeshBoundary";
    case SceneComponentKind::Emitter:       return "Emitter";
    case SceneComponentKind::OutflowRegion: return "OutflowRegion";
    }
    return "?";
}

} // namespace Phantom
