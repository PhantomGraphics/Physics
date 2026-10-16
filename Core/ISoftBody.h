#pragma once

#include "SoftMesh.h"
#include "XPBDSolver.h"

namespace Phantom {
namespace Physics {

struct ISoftBody {
    virtual void            build(XPBDSolver& solver) = 0;
    virtual SoftMesh&       getMesh()                 = 0;
    virtual const SoftMesh& getMesh() const           = 0;
    virtual ~ISoftBody()                              = default;
};

} // namespace Physics
} // namespace Phantom
