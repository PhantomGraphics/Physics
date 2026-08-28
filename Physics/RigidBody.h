#pragma once

#include "ICollisionShape.h"
#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Matrix3d.h"
#include "CGLib/Math/Quaternion.h"
#include "CGLib/Math/Box3d.h"
#include "CGLib/Util/UnCopyable.h"

namespace Phantom {
namespace Physics {

class RigidBody : private UnCopyable {
public:
    // --- state (integrated) ---
    Math::Vector3df  position        = { 0.f, 0.f, 0.f };
    Math::Quaternion orientation      = { 1.f, 0.f, 0.f, 0.f };
    Math::Vector3df  linearVelocity  = { 0.f, 0.f, 0.f };
    Math::Vector3df  angularVelocity = { 0.f, 0.f, 0.f };

    // --- material ---
    float            mass        = 1.0f;
    float            inverseMass = 1.0f;
    float            restitution = 0.3f;
    float            friction    = 0.5f;

    // --- shape (non-owning) ---
    ICollisionShape* shape = nullptr;

    // --- force accumulators (reset each step) ---
    // Explicitly zeroed: Math::Vector3df is glm::vec3, and this build does not
    // define GLM_FORCE_CTOR_INIT, so a defaulted member would be indeterminate
    // until the first integrate()/RigidBodySolver::reset() clears it -- a body
    // stepped straight after construction (SetPreset without a following Reset)
    // would otherwise take a garbage force on its very first step.
    Math::Vector3df  forceAccum  = { 0.f, 0.f, 0.f };
    Math::Vector3df  torqueAccum = { 0.f, 0.f, 0.f };

    bool isStatic() const { return inverseMass == 0.f; }

    void setMass(float m);
    void setShape(ICollisionShape* s) { shape = s; }

    void applyForce (const Math::Vector3df& f) { forceAccum  += f; }
    void applyTorque(const Math::Vector3df& t) { torqueAccum += t; }
    void applyImpulse(const Math::Vector3df& impulse,
                      const Math::Vector3df& worldPoint);

    void integrate(float dt);
    void updateInertiaTensor();

    Math::Matrix3df getInverseInertiaTensorWorld() const { return invI_world_; }
    Math::Box3df    getAABB()                      const;

private:
    Math::Matrix3df invI_local_ = Math::Matrix3df(0.f);
    Math::Matrix3df invI_world_ = Math::Matrix3df(0.f);
    Math::Matrix3df R_          = Math::Matrix3df(1.f);

    void computeLocalInertia();
};

} // namespace Physics
} // namespace Phantom
