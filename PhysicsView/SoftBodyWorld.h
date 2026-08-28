#pragma once

#include "Physics/Physics/SoftBodySolver.h"
#include "Physics/Physics/PhysicsSolver.h"
#include "Physics/Physics/ISoftBody.h"
#include "Physics/Physics/ClothBody.h"
#include "Physics/Physics/RopeBody.h"
#include "Physics/Physics/JellyBody.h"
#include "Physics/Physics/RigidBody.h"
#include "Physics/Physics/ICollisionShape.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace Phantom {

enum class SoftBodyPreset {
    ClothTwoPin,
    ClothTopEdge,
    ClothWithSphere,
    ClothOnBox,
    JellySelfOverlap,
    RopeHanging,
    RopePendulum,
    RopeBothEndsPinned,
    JellyDrop,
    JellyOnBox,
    TwoJelliesStacked,
    Mixed,
};

class SoftBodyWorld {
public:
    struct WireData {
        std::vector<float>    positions;
        std::vector<float>    colors;
        std::vector<uint32_t> indices;
    };

    // solver must outlive this SoftBodyWorld (typically owned by the same FluidWorld that
    // constructs this). SoftBody-Fluid coupling (mirrors RigidBodyWorld) reaches
    // solver.softFluidSolver() directly from FluidWorld rather than through this class.
    explicit SoftBodyWorld(Physics::PhysicsSolver& solver);

    void           setPreset(SoftBodyPreset p);
    SoftBodyPreset currentPreset() const { return preset_; }

    void step();
    void reset();
    bool isRunning()        const;
    void setRunning(bool v);

    Physics::SoftBodySolver&       getWorld()       { return physicsSolver_.softSolver(); }
    const Physics::SoftBodySolver& getWorld() const { return physicsSolver_.softSolver(); }

    /** @brief Advances one step regardless of isRunning() (see RigidBodyWorld::stepForced()). */
    void stepForced();

    // Non-owning pointers to every body created by the current preset, so a
    // higher-level world (FluidWorld) can enumerate them for coupling --
    // SoftBodySolver itself does not expose a getBodies() accessor.
    const std::vector<Physics::ISoftBody*>& getBodyPointers() const { return bodyPtrs_; }

    // Aggregate Y-position extents across all current bodies' particles
    // (regression/coupling verification use, mirrors FluidWorld's
    // Get{Max,Min}ParticlePositionY). Returns 0 if there are no particles.
    float getMinParticlePositionY() const;
    float getMaxParticlePositionY() const;

    // Y position of a single particle in bodyPtrs_[bodyIndex] (scenario-test
    // precision checks, e.g. "did this pinned particle actually not move").
    // Returns 0 for an out-of-range bodyIndex/particleIndex.
    float getParticlePositionY(int bodyIndex, int particleIndex) const;

    // Total volume of bodyPtrs_[bodyIndex] if it is a JellyBody (0 otherwise
    // or if bodyIndex is out of range) -- volume conservation is only a
    // meaningful concept for a volumetric (tetrahedral) body.
    float getBodyTotalVolume(int bodyIndex) const;

    // Minimum pairwise particle distance between bodyPtrs_[bodyIndexA] and
    // bodyPtrs_[bodyIndexB] (cross-body collision regression checks).
    // Returns 0 if either index is out of range.
    float getMinInterBodyDistance(int bodyIndexA, int bodyIndexB) const;

    // Minimum pairwise particle distance *within* bodyPtrs_[bodyIndex],
    // excluding pairs directly joined by a mesh.edges entry (structural or
    // bending -- the same exclusion SelfCollision::update() applies, see
    // Physics/SelfCollision.cpp). Distinguishes "self-collision kept the mesh
    // from folding onto itself" from "adjacent, structurally-constrained
    // particles are naturally close" (unlike getMinInterBodyDistance(i, i),
    // which trivially returns 0 -- every particle is distance 0 from itself).
    // Returns a large sentinel if bodyIndex is out of range or has <2 particles.
    float getMinNonEdgeDistance(int bodyIndex) const;

    WireData buildWireData() const;

private:
    Physics::PhysicsSolver& physicsSolver_;
    SoftBodyPreset         preset_ = SoftBodyPreset::ClothTwoPin;
    std::vector<Physics::ISoftBody*> bodyPtrs_;

    // SoftBodySolver only holds non-owning pointers (see Physics/Physics/SoftBodySolver.h);
    // this world owns every ISoftBody it creates.
    std::vector<std::unique_ptr<Physics::ISoftBody>> ownedBodies_;

    // Constructs a T, keeps it alive in ownedBodies_, and registers it with both the solver
    // and bodyPtrs_ in one call (used by applyPreset()).
    template<typename T, typename... Args>
    T* spawnBody(Args&&... args) {
        auto owned = std::make_unique<T>(std::forward<Args>(args)...);
        T* ptr = owned.get();
        ownedBodies_.push_back(std::move(owned));
        getWorld().addBody(ptr);
        bodyPtrs_.push_back(ptr);
        return ptr;
    }

    // JellyOnBox / ClothOnBox / Mixed 用の静的 RigidBody（非所有ポインタとして world_ に登録）。
    // integrate() は呼ばないため、常に position/orientation 固定の静的コライダーとして働く。
    Physics::BoxShape  rigidBoxShape_;
    Physics::RigidBody rigidBox_;

    void applyPreset();
};

} // namespace Phantom
