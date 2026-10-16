#pragma once

#include "ISoftCollider.h"

namespace Phantom {
namespace Physics {

class RigidBody;

// Physics/Fluid の RigidBody (ICollisionShape ベースの SDF) を
// ISoftCollider として包むアダプタ。RigidBody は非所有（外部の
// RigidBodyWorld 等が生存期間を管理する）。
class RigidBodyCollider : public ISoftCollider {
public:
    explicit RigidBodyCollider(RigidBody* body) : body_(body) {}

    void resolve(SoftParticleSoA& particles, size_t i) const override;

    RigidBody* getBody() const { return body_; }

private:
    RigidBody* body_ = nullptr;
};

} // namespace Physics
} // namespace Phantom
