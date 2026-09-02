#pragma once

#include "Physics/Physics/RigidBody.h"
#include "Physics/Physics/RigidBodySolver.h"
#include "Physics/Physics/PhysicsSolver.h"
#include "Physics/Physics/ICollisionShape.h"
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace Phantom {

enum class ScenePreset {
    SphereDrop,
    BoxDrop,
    Stacking,
    NewtonsCradle,
    Billiards,
    SphereBoxCollision,
    Custom,
};

class RigidBodyWorld {
public:
    // solver must outlive this RigidBodyWorld (typically owned by the same FluidWorld that
    // constructs this). Rigid-Fluid coupling (internal design notes) reaches
    // solver.rigidFluidSolver() directly from FluidWorld rather than through this class -- see
    // its class doc.
    explicit RigidBodyWorld(Physics::PhysicsSolver& solver);

    void setPreset(ScenePreset preset);
    void reset();
    void step();

    /** @brief Advances one step regardless of isRunning() (see RigidBodyWorld::stepUnconditional()). */
    void stepForced();

    bool isRunning() const  { return physicsSolver_.rigidSolver().isRunning(); }
    void setRunning(bool v) { physicsSolver_.rigidSolver().setRunning(v); }

    ScenePreset currentPreset() const { return preset_; }

    Physics::RigidBody* addSphere(const Math::Vector3df& pos, float radius, float mass,
                                  float restitution = 0.3f, float friction = 0.5f,
                                  const Math::Vector3df& vel = {});
    Physics::RigidBody* addBox(const Math::Vector3df& pos, const Math::Vector3df& halfExtents,
                               float mass,
                               float restitution = 0.3f, float friction = 0.5f,
                               const Math::Quaternion& orient = {1,0,0,0},
                               const Math::Vector3df& vel = {});
    void addFloor(float y = 0.f);

    const Physics::RigidBodySolver& getWorld() const { return physicsSolver_.rigidSolver(); }
    Physics::RigidBodySolver&       getWorld()       { return physicsSolver_.rigidSolver(); }

    struct WireData {
        std::vector<float>    positions;
        std::vector<float>    colors;
        std::vector<uint32_t> indices;
    };
    WireData buildWireData() const;

private:
    Physics::PhysicsSolver& physicsSolver_;
    ScenePreset              preset_ = ScenePreset::SphereDrop;

    // RigidBodySolver only holds non-owning pointers (see Physics/Physics/RigidBodySolver.h);
    // this world owns every RigidBody it creates, and (since each body owns its shape
    // 1:1 and no shape is ever shared between bodies) the ICollisionShape backing its
    // RigidBody::shape pointer too.
    std::vector<std::unique_ptr<Physics::RigidBody>>       bodies_;
    std::vector<std::unique_ptr<Physics::ICollisionShape>> shapes_;

    void buildPreset();

    static void emitVertex(WireData& wd, const glm::vec3& p, const glm::vec4& c);
    static void buildSphereWire(WireData& wd, const glm::vec3& center, float radius, const glm::vec4& c);
    static void buildBoxWire(WireData& wd, const Physics::BoxShape& shape,
                             const Math::Vector3df& pos, const Math::Quaternion& orient,
                             const glm::vec4& c);
    static void buildFloorWire(WireData& wd, float y);
};

} // namespace Phantom
