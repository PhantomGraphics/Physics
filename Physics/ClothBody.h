#pragma once

#include "ISoftBody.h"
#include "CGLib/Math/Vector3d.h"

namespace Phantom {
namespace Physics {

struct ClothBodyParams {
    int             rows        = 20;
    int             cols        = 20;
    float           width       = 2.f;
    float           height      = 2.f;
    float           distAlpha   = 0.f;     // stretch compliance (0 = rigid)
    float           bendAlpha   = 1e-7f;   // bend compliance
    bool            pinTopLeft  = true;
    bool            pinTopRight = true;
    bool            pinTopEdge  = false;
    Math::Vector3df origin      = { 0.f, 0.f, 0.f };  // 生成位置のオフセット
};

class ClothBody : public ISoftBody {
public:
    explicit ClothBody(const ClothBodyParams& p = {});

    void            build(XPBDSolver& solver) override;
    SoftMesh&       getMesh()                 override { return mesh_; }
    const SoftMesh& getMesh()          const  override { return mesh_; }

    float getStretch(int edgeIdx) const;

private:
    ClothBodyParams params_;
    SoftMesh        mesh_;

    float computeRestCos(int i0, int i1, int i2, int i3) const;
};

} // namespace Physics
} // namespace Phantom
