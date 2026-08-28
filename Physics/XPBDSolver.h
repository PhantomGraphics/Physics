#pragma once

#include <memory>
#include <vector>
#include "CGLib/Math/Vector3d.h"
#include "CGLib/Util/UnCopyable.h"
#include "SoftMesh.h"
#include "IConstraint.h"
#include "ISoftCollider.h"
#include "SelfCollision.h"

namespace Phantom {
namespace Physics {

class XPBDSolver : private UnCopyable {
public:
    struct Params {
        float           timeStep      = 0.016f;
        int             numSubsteps   = 10;
        int             numIterations = 5;
        float           damping       = 0.99f;
        Math::Vector3df gravity       = { 0.f, -9.8f, 0.f };
        Math::Vector3df wind          = {};

        bool            selfCollisionEnabled   = false;
        float           selfCollisionThickness = 0.02f;
        float           selfCollisionCellSize  = 0.1f;
    };

    void   setMesh(SoftMesh* mesh);
    void   addConstraint(std::unique_ptr<IConstraint> c);
    void   addCollider(ISoftCollider* c);
    void   clearConstraints();
    void   clearColliders() { colliders_.clear(); }
    size_t getConstraintCount() const { return constraints_.size(); }

    Params&       params()       { return params_; }
    const Params& params() const { return params_; }

    void step();
    void reset();

private:
    SoftMesh*                                   mesh_     = nullptr;
    Params                                      params_;
    std::vector<std::unique_ptr<IConstraint>>   constraints_;
    std::vector<ISoftCollider*>                 colliders_;
    std::vector<Math::Vector3df>                restPositions_;
    SelfCollision                               selfCollision_;

    void applyExternalForces(float dt);
    void predictPositions(float dt);
    void projectConstraints(float dt);
    void updateVelocities(float dt);
    void resolveCollisions();
    void resolveSelfCollision();
};

} // namespace Physics
} // namespace Phantom
