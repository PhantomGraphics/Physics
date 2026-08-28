#include "SoftBodySolver.h"
#include "Physics/Physics/RigidBody.h"
#include "Physics/Physics/ICollisionShape.h"

#include <array>
#include <cmath>

namespace Phantom {
namespace Physics {

namespace {

// center を中心とする半径 radius の円（axis1-axis2 平面）をラインリストとして追加
void appendCircleWire(SoftBodySolver::WireData& data,
                       const Math::Vector3df& center, float radius,
                       const glm::vec3& axis1, const glm::vec3& axis2,
                       const glm::vec4& color, int segments = 32) {
    for (int i = 0; i < segments; ++i) {
        constexpr float kTwoPi = 6.28318530717958647692f;
        float t0 = kTwoPi * static_cast<float>(i)     / static_cast<float>(segments);
        float t1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        glm::vec3 p0 = glm::vec3(center) + radius * (std::cos(t0) * axis1 + std::sin(t0) * axis2);
        glm::vec3 p1 = glm::vec3(center) + radius * (std::cos(t1) * axis1 + std::sin(t1) * axis2);
        data.positions.push_back(p0);
        data.positions.push_back(p1);
        data.colors.push_back(color);
        data.colors.push_back(color);
    }
}

// 3 枚の直交する大円でワイヤーフレーム球を表現する
void appendSphereWire(SoftBodySolver::WireData& data,
                       const Math::Vector3df& center, float radius) {
    const glm::vec4 color{ 0.7f, 0.7f, 0.8f, 1.f };
    appendCircleWire(data, center, radius, { 1.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, color);  // XY
    appendCircleWire(data, center, radius, { 1.f, 0.f, 0.f }, { 0.f, 0.f, 1.f }, color);  // XZ
    appendCircleWire(data, center, radius, { 0.f, 1.f, 0.f }, { 0.f, 0.f, 1.f }, color);  // YZ
}

// 8 頂点（BoxShape::getWorldCorners の並び）から 12 辺のワイヤーボックスを追加する
void appendBoxWire(SoftBodySolver::WireData& data,
                    const std::array<Math::Vector3df, 8>& c) {
    static constexpr int edges[12][2] = {
        { 0, 1 }, { 0, 2 }, { 0, 4 }, { 1, 3 }, { 1, 5 }, { 2, 3 },
        { 2, 6 }, { 3, 7 }, { 4, 5 }, { 4, 6 }, { 5, 7 }, { 6, 7 },
    };
    const glm::vec4 color{ 0.6f, 0.5f, 0.9f, 1.f };
    for (const auto& e : edges) {
        data.positions.push_back(glm::vec3(c[e[0]]));
        data.positions.push_back(glm::vec3(c[e[1]]));
        data.colors.push_back(color);
        data.colors.push_back(color);
    }
}

} // namespace

void SoftBodySolver::clearBodies() {
    bodies_.clear();
    solvers_.clear();
}

void SoftBodySolver::addRigidBodyCollider(RigidBody* rb) {
    rigidColliders_.push_back(std::make_unique<RigidBodyCollider>(rb));
    syncColliders();
}

void SoftBodySolver::clearRigidBodyColliders() {
    rigidColliders_.clear();
    syncColliders();
}

Math::Vector3df SoftBodySolver::computeRigidReactionForce(const RigidBoundary& boundary,
                                                            const Math::Vector3df& pivot,
                                                            Math::Vector3df& outTorque) const {
    Math::Vector3df totalForce{ 0.f, 0.f, 0.f };
    outTorque = { 0.f, 0.f, 0.f };

    for (const auto& body : bodies_) {
        const auto& particles = body->getMesh().particles;
        for (size_t i = 0; i < particles.size(); ++i) {
            // getBoundaryForce() returns the penalty spring's force on the *particle* (pushing
            // it away from the rigid shape's center, along the surface normal at its position).
            // Newton's third law: the rigid body feels the equal-and-opposite reaction.
            const Math::Vector3df& pos = particles.positions[i];
            const Math::Vector3df reaction = -boundary.getBoundaryForce(pos);
            totalForce += reaction;
            outTorque  += glm::cross(pos - pivot, reaction);
        }
    }
    return totalForce;
}

void SoftBodySolver::step() {
    if (!params_.running) return;
    stepInternal();
}

void SoftBodySolver::stepUnconditional() {
    stepInternal();
}

void SoftBodySolver::stepInternal() {
    syncColliders();
    for (size_t i = 0; i < solvers_.size(); ++i) {
        solvers_[i]->params() = solverParams_;
        solvers_[i]->step();
    }

    if (params_.crossBodyCollisionEnabled && bodies_.size() > 1) {
        std::vector<SoftMesh*> meshes;
        meshes.reserve(bodies_.size());
        for (auto& b : bodies_)
            meshes.push_back(&b->getMesh());
        crossBodyCollision_.resolve(meshes,
            { params_.crossBodyCollisionThickness, params_.crossBodyCollisionCellSize },
            solverParams_.timeStep);
    }
}

void SoftBodySolver::reset() {
    for (auto& s : solvers_)
        s->reset();
}

size_t SoftBodySolver::getParticleCount() const {
    size_t n = 0;
    for (const auto& b : bodies_)
        n += b->getMesh().particles.size();
    return n;
}

float SoftBodySolver::getMaxSpeed() const {
    float m = 0.f;
    for (const auto& b : bodies_)
        for (const auto& v : b->getMesh().particles.velocities)
            m = std::max(m, glm::length(v));
    return m;
}

SoftBodySolver::WireData SoftBodySolver::buildWireData() const {
    WireData data;
    for (const auto& body : bodies_) {
        const auto& mesh = body->getMesh();
        for (const auto& edge : mesh.edges) {
            const auto& pa = mesh.particles.positions[edge.a];
            const auto& pb = mesh.particles.positions[edge.b];
            data.positions.push_back(glm::vec3(pa));
            data.positions.push_back(glm::vec3(pb));

            float stretch = (edge.restLength > 1e-8f)
                ? glm::length(pb - pa) / edge.restLength : 1.f;
            float r = glm::clamp(stretch - 1.f, 0.f, 1.f);
            float b = glm::clamp(1.f - stretch, 0.f, 1.f);
            glm::vec4 col = { r, 1.f - r - b, b, 1.f };
            data.colors.push_back(col);
            data.colors.push_back(col);
        }
    }

    if (params_.sphereEnabled)
        appendSphereWire(data, params_.sphereCenter, params_.sphereRadius);

    for (const auto& rc : rigidColliders_) {
        RigidBody* rb = rc->getBody();
        if (!rb || !rb->shape) continue;
        if (rb->shape->getType() == ShapeType::Box) {
            auto* box = static_cast<BoxShape*>(rb->shape);
            appendBoxWire(data, box->getWorldCorners(rb->position, rb->orientation));
        }
    }

    return data;
}

void SoftBodySolver::rebuildEntry(size_t idx) {
    auto& solver = *solvers_[idx];
    auto& body   = *bodies_[idx];
    solver.params() = solverParams_;
    solver.clearConstraints();
    solver.setMesh(&body.getMesh());
    body.build(solver);
    syncColliders();
}

void SoftBodySolver::syncColliders() {
    sphere_.center = params_.sphereCenter;
    sphere_.radius = params_.sphereRadius;
    floor_.normal  = { 0.f, 1.f, 0.f };
    floor_.offset  = 0.f;

    for (auto& s : solvers_) {
        s->clearColliders();
        s->addCollider(&floor_);
        if (params_.sphereEnabled)
            s->addCollider(&sphere_);
        for (auto& rc : rigidColliders_)
            s->addCollider(rc.get());
    }
}

} // namespace Physics
} // namespace Phantom
