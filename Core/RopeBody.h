#pragma once

#include "ISoftBody.h"

namespace Phantom {
namespace Physics {

struct RopeBodyParams {
    int   n         = 30;
    float length    = 3.f;
    bool  pinStart  = true;
    bool  pinEnd    = false;
    float distAlpha = 0.f;     // stretch compliance (0 = rigid)
    float bendAlpha = 1e-6f;   // bend compliance
};

class RopeBody : public ISoftBody {
public:
    explicit RopeBody(const RopeBodyParams& p = {});

    void            build(XPBDSolver& solver) override;
    SoftMesh&       getMesh()                 override { return mesh_; }
    const SoftMesh& getMesh()          const  override { return mesh_; }

private:
    RopeBodyParams params_;
    SoftMesh       mesh_;
};

} // namespace Physics
} // namespace Phantom
