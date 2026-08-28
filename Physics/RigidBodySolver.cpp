#include "pch.h"
#include "RigidBodySolver.h"
#include "BroadPhase.h"
#include "NarrowPhase.h"

namespace Phantom {
namespace Physics {

void RigidBodySolver::clearBodies() {
    bodies_.clear();
    contacts_.clear();
    snapshots_.clear();
    running_ = false;
}

void RigidBodySolver::saveSnapshot() {
    snapshots_.clear();
    for (const auto& b : bodies_) {
        snapshots_.push_back({
            b->position,
            b->orientation,
            b->linearVelocity,
            b->angularVelocity
        });
    }
}

void RigidBodySolver::reset() {
    for (size_t i = 0; i < bodies_.size() && i < snapshots_.size(); ++i) {
        bodies_[i]->position        = snapshots_[i].position;
        bodies_[i]->orientation     = snapshots_[i].orientation;
        bodies_[i]->linearVelocity  = snapshots_[i].linearVelocity;
        bodies_[i]->angularVelocity = snapshots_[i].angularVelocity;
        bodies_[i]->forceAccum      = Math::Vector3df(0.f);
        bodies_[i]->torqueAccum     = Math::Vector3df(0.f);
    }
    contacts_.clear();
    updateInertias();
}

void RigidBodySolver::step() {
    if (!running_) return;
    stepUnconditional();
}

void RigidBodySolver::stepUnconditional() {
    float dt = timeStep;

    updateInertias();
    applyGravity(dt);
    integratePositions(dt);

    std::vector<std::pair<int, int>> pairs;
    broadPhase(pairs);

    contacts_.clear();
    narrowPhase(pairs);
    prepareContacts();
    solveContacts(dt);
}

// ----------------------------------------------------------------- private --

void RigidBodySolver::updateInertias() {
    for (auto& b : bodies_)
        b->updateInertiaTensor();
}

void RigidBodySolver::applyGravity(float dt) {
    for (auto& b : bodies_) {
        if (!b->isStatic())
            b->linearVelocity += params_.gravity * dt;
    }
}

void RigidBodySolver::integratePositions(float dt) {
    for (auto& b : bodies_)
        b->integrate(dt);
}

void RigidBodySolver::broadPhase(std::vector<std::pair<int, int>>& pairs) {
    BroadPhase::detect(bodies_, pairs);
}

void RigidBodySolver::narrowPhase(const std::vector<std::pair<int, int>>& pairs) {
    for (auto& [ia, ib] : pairs) {
        ContactManifold manifold;
        if (NarrowPhase::detect(*bodies_[ia], *bodies_[ib], manifold) &&
            !manifold.contacts.empty())
        {
            contacts_.push_back(std::move(manifold));
        }
    }
}

void RigidBodySolver::prepareContacts() {
    for (auto& manifold : contacts_) {
        RigidBody* a = manifold.bodyA;
        RigidBody* b = manifold.bodyB;
        for (auto& cp : manifold.contacts) {
            Math::Vector3df rA  = cp.position - a->position;
            Math::Vector3df rB  = cp.position - b->position;
            Math::Vector3df vA  = a->linearVelocity + glm::cross(a->angularVelocity, rA);
            Math::Vector3df vB  = b->linearVelocity + glm::cross(b->angularVelocity, rB);
            float vn = glm::dot(vA - vB, cp.normal);
            cp.initialVelocity  = (vn < 0.f) ? -vn : 0.f;
            cp.accImpulse       = 0.f;
            cp.accFriction      = 0.f;
        }
    }
}

void RigidBodySolver::solveContacts(float dt) {
    for (int iter = 0; iter < params_.solverIterations; ++iter) {
        for (auto& manifold : contacts_) {
            RigidBody* a  = manifold.bodyA;
            RigidBody* b  = manifold.bodyB;
            auto invIA = a->getInverseInertiaTensorWorld();
            auto invIB = b->getInverseInertiaTensorWorld();
            float e   = std::min(a->restitution, b->restitution);
            float mu  = (a->friction + b->friction) * 0.5f;

            for (auto& cp : manifold.contacts) {
                Math::Vector3df rA = cp.position - a->position;
                Math::Vector3df rB = cp.position - b->position;

                Math::Vector3df vA  = a->linearVelocity + glm::cross(a->angularVelocity, rA);
                Math::Vector3df vB  = b->linearVelocity + glm::cross(b->angularVelocity, rB);
                Math::Vector3df vRel = vA - vB;
                float vn = glm::dot(vRel, cp.normal);

                float bias = params_.baumgarteBeta / dt
                           * std::max(cp.penetration - params_.slop, 0.f);

                float K = a->inverseMass + b->inverseMass
                    + glm::dot(cp.normal, glm::cross(invIA * glm::cross(rA, cp.normal), rA))
                    + glm::dot(cp.normal, glm::cross(invIB * glm::cross(rB, cp.normal), rB));

                if (K < 1e-9f) continue;

                float lambda = -(vn - e * cp.initialVelocity - bias) / K;

                float newAcc = std::max(cp.accImpulse + lambda, 0.f);
                lambda        = newAcc - cp.accImpulse;
                cp.accImpulse = newAcc;

                Math::Vector3df J = lambda * cp.normal;
                a->applyImpulse( J, cp.position);
                b->applyImpulse(-J, cp.position);

                // Friction
                vA   = a->linearVelocity + glm::cross(a->angularVelocity, rA);
                vB   = b->linearVelocity + glm::cross(b->angularVelocity, rB);
                vRel = vA - vB;

                Math::Vector3df vTangent = vRel - glm::dot(vRel, cp.normal) * cp.normal;
                float vt = glm::length(vTangent);

                if (vt > 1e-6f && mu > 0.f) {
                    Math::Vector3df tangent = vTangent / vt;
                    float Kt = a->inverseMass + b->inverseMass
                        + glm::dot(tangent, glm::cross(invIA * glm::cross(rA, tangent), rA))
                        + glm::dot(tangent, glm::cross(invIB * glm::cross(rB, tangent), rB));

                    if (Kt > 1e-9f) {
                        float lambdaF  = -vt / Kt;
                        float maxF     = mu * cp.accImpulse;
                        float newAccF  = std::max(-maxF, std::min(cp.accFriction + lambdaF, maxF));
                        lambdaF        = newAccF - cp.accFriction;
                        cp.accFriction = newAccF;

                        Math::Vector3df Jf = lambdaF * tangent;
                        a->applyImpulse( Jf, cp.position);
                        b->applyImpulse(-Jf, cp.position);
                    }
                }
            }
        }
    }
}

} // namespace Physics
} // namespace Phantom
