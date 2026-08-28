#pragma once

#include "ISoftBody.h"
#include "CGLib/Math/Vector3d.h"

namespace Phantom {
namespace Physics {

struct JellyBodyParams {
    int   nx = 3, ny = 3, nz = 3;
    float w = 1.0f, h = 1.0f, d = 1.0f;
    Math::Vector3df offset = { 0.f, 1.f, 0.f };

    float distAlpha   = 1e-5f;
    float volumeAlpha = 1e-4f;
};

class JellyBody : public ISoftBody {
public:
    explicit JellyBody(const JellyBodyParams& p = {});

    void            build(XPBDSolver& solver) override;
    SoftMesh&       getMesh()                 override { return mesh_; }
    const SoftMesh& getMesh()          const  override { return mesh_; }

    // 全テトラの符号付き体積の総和（現在位置ベース）
    float getTotalVolume() const;

private:
    JellyBodyParams params_;
    SoftMesh        mesh_;
};

} // namespace Physics
} // namespace Phantom
