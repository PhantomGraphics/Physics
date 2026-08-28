#include "pch.h"
#include "RigidBodyWorld.h"

#include <unordered_set>
#include <cmath>

namespace Phantom {

static constexpr float kPi = 3.14159265f;

RigidBodyWorld::RigidBodyWorld(Physics::PhysicsSolver& solver) : physicsSolver_(solver) {
    setPreset(ScenePreset::SphereDrop);
}

void RigidBodyWorld::setPreset(ScenePreset preset) {
    physicsSolver_.rigidSolver().clear();
    bodies_.clear();
    shapes_.clear();
    preset_ = preset;
    buildPreset();
    physicsSolver_.rigidSolver().saveSnapshot();
}

void RigidBodyWorld::reset() {
    physicsSolver_.rigidSolver().reset();
}

void RigidBodyWorld::step() {
    physicsSolver_.rigidSolver().step();
}

void RigidBodyWorld::stepForced() {
    physicsSolver_.rigidSolver().stepUnconditional();
}

Physics::RigidBody* RigidBodyWorld::addSphere(const Math::Vector3df& pos, float radius,
                                                    float mass, float restitution, float friction,
                                                    const Math::Vector3df& vel)
{
    auto shape = std::make_unique<Physics::SphereShape>();
    shape->radius = radius;
    Physics::ICollisionShape* shapePtr = shape.get();
    shapes_.push_back(std::move(shape));

    auto body = std::make_unique<Physics::RigidBody>();
    body->position       = pos;
    body->linearVelocity = vel;
    body->setShape(shapePtr);
    body->setMass(mass);
    body->restitution = restitution;
    body->friction    = friction;
    Physics::RigidBody* ptr = body.get();
    bodies_.push_back(std::move(body));
    physicsSolver_.rigidSolver().addBody(ptr);
    // Re-snapshot so Reset() (RigidBodySolver::reset() restores by index up
    // to snapshots_.size(), see its doc comment) knows about this body too --
    // otherwise a body added after setPreset()'s own saveSnapshot() (e.g. via
    // AddSphere on a Custom scene) is silently skipped by Reset() and keeps
    // whatever position the simulation carried it to (docs/issue/CODEBASE_ISSUES.md 1.4).
    physicsSolver_.rigidSolver().saveSnapshot();
    return ptr;
}

Physics::RigidBody* RigidBodyWorld::addBox(const Math::Vector3df& pos,
                                                  const Math::Vector3df& halfExtents,
                                                  float mass, float restitution, float friction,
                                                  const Math::Quaternion& orient,
                                                  const Math::Vector3df& vel)
{
    auto shape = std::make_unique<Physics::BoxShape>();
    shape->halfExtents = halfExtents;
    Physics::ICollisionShape* shapePtr = shape.get();
    shapes_.push_back(std::move(shape));

    auto body = std::make_unique<Physics::RigidBody>();
    body->position       = pos;
    body->orientation    = orient;
    body->linearVelocity = vel;
    body->setShape(shapePtr);
    body->setMass(mass);
    body->restitution = restitution;
    body->friction    = friction;
    Physics::RigidBody* ptr = body.get();
    bodies_.push_back(std::move(body));
    physicsSolver_.rigidSolver().addBody(ptr);
    physicsSolver_.rigidSolver().saveSnapshot();  // see addSphere()'s comment above
    return ptr;
}

void RigidBodyWorld::addFloor(float y) {
    auto shape = std::make_unique<Physics::PlaneShape>();
    shape->normal = { 0.f, 1.f, 0.f };
    shape->offset = y;
    Physics::ICollisionShape* shapePtr = shape.get();
    shapes_.push_back(std::move(shape));

    auto body = std::make_unique<Physics::RigidBody>();
    body->position = { 0.f, y, 0.f };
    body->setShape(shapePtr);
    body->setMass(0.f);
    body->restitution = 0.f;
    body->friction    = 0.5f;
    Physics::RigidBody* ptr = body.get();
    bodies_.push_back(std::move(body));
    physicsSolver_.rigidSolver().addBody(ptr);
    physicsSolver_.rigidSolver().saveSnapshot();  // see addSphere()'s comment above
}

void RigidBodyWorld::buildPreset() {
    switch (preset_) {
    case ScenePreset::SphereDrop:
        addFloor(0.f);
        addSphere({0.f, 3.f, 0.f}, 0.5f, 1.f);
        break;

    case ScenePreset::BoxDrop:
        addFloor(0.f);
        addBox({0.f, 3.f, 0.f}, {0.5f, 0.5f, 0.5f}, 1.f);
        break;

    case ScenePreset::Stacking:
        addFloor(0.f);
        for (int i = 0; i < 5; ++i)
            addBox({0.f, 0.5f + i * 1.0f, 0.f}, {0.5f, 0.5f, 0.5f}, 1.f);
        break;

    case ScenePreset::NewtonsCradle: {
        addFloor(0.f);
        const float spacing = 1.02f;
        for (int i = 0; i < 5; ++i) {
            float x = (i - 2) * spacing;
            Math::Vector3df vel = (i == 4)
                ? Math::Vector3df{-5.f, 0.f, 0.f}
                : Math::Vector3df{};
            addSphere({x, 0.5f, 0.f}, 0.5f, 1.f, 1.0f, 0.0f, vel);
        }
        break;
    }

    case ScenePreset::Billiards: {
        addFloor(0.f);
        addSphere({-4.f, 0.5f, 0.f}, 0.5f, 1.f, 0.9f, 0.1f, {6.f, 0.f, 0.f});
        const float sp = 1.02f;
        for (int i = 0; i < 5; ++i)
            addSphere({i * sp, 0.5f, 0.f}, 0.5f, 1.f, 0.9f, 0.1f);
        break;
    }

    case ScenePreset::SphereBoxCollision:
        addFloor(0.f);
        addBox({0.f, 0.5f, 0.f}, {0.5f, 0.5f, 0.5f}, 2.f, 0.3f, 0.4f);
        addSphere({-3.f, 1.f, 0.f}, 0.3f, 0.5f, 0.5f, 0.3f, {4.f, 0.f, 0.f});
        break;

    case ScenePreset::Custom:
        addFloor(0.f);
        break;
    }
}

// ---- WireData building -------------------------------------------------------

void RigidBodyWorld::emitVertex(WireData& wd, const glm::vec3& p, const glm::vec4& c) {
    wd.positions.push_back(p.x);
    wd.positions.push_back(p.y);
    wd.positions.push_back(p.z);
    wd.colors.push_back(c.r);
    wd.colors.push_back(c.g);
    wd.colors.push_back(c.b);
    wd.colors.push_back(c.a);
}

void RigidBodyWorld::buildSphereWire(WireData& wd, const glm::vec3& center,
                                           float radius, const glm::vec4& c)
{
    const int   N    = 32;
    const float step = 2.f * kPi / N;

    for (int circle = 0; circle < 3; ++circle) {
        uint32_t base = static_cast<uint32_t>(wd.positions.size() / 3);
        for (int i = 0; i < N; ++i) {
            float ct = std::cos(i * step);
            float st = std::sin(i * step);
            glm::vec3 p;
            switch (circle) {
            case 0: p = center + glm::vec3(radius*ct, radius*st, 0.f); break;
            case 1: p = center + glm::vec3(radius*ct, 0.f, radius*st); break;
            default:p = center + glm::vec3(0.f, radius*ct, radius*st); break;
            }
            emitVertex(wd, p, c);
        }
        for (int i = 0; i < N; ++i) {
            wd.indices.push_back(base + i);
            wd.indices.push_back(base + (i + 1) % N);
        }
    }
}

void RigidBodyWorld::buildBoxWire(WireData& wd, const Physics::BoxShape& shape,
                                        const Math::Vector3df& pos,
                                        const Math::Quaternion& orient,
                                        const glm::vec4& c)
{
    auto corners = shape.getWorldCorners(pos, orient);
    uint32_t base = static_cast<uint32_t>(wd.positions.size() / 3);

    for (const auto& corner : corners)
        emitVertex(wd, glm::vec3(corner.x, corner.y, corner.z), c);

    static const uint32_t kEdges[12][2] = {
        {0,4},{1,5},{2,6},{3,7},
        {0,2},{1,3},{4,6},{5,7},
        {0,1},{2,3},{4,5},{6,7}
    };
    for (const auto& e : kEdges) {
        wd.indices.push_back(base + e[0]);
        wd.indices.push_back(base + e[1]);
    }
}

void RigidBodyWorld::buildFloorWire(WireData& wd, float y) {
    const float gridSize = 8.f;
    const int   N        = 17;
    const float step     = 2.f * gridSize / (N - 1);
    const glm::vec4 c    = {1.f, 1.f, 1.f, 0.4f};

    for (int i = 0; i < N; ++i) {
        float x = -gridSize + i * step;
        uint32_t base = static_cast<uint32_t>(wd.positions.size() / 3);
        emitVertex(wd, {x, y, -gridSize}, c);
        emitVertex(wd, {x, y,  gridSize}, c);
        wd.indices.push_back(base);
        wd.indices.push_back(base + 1);
    }
    for (int i = 0; i < N; ++i) {
        float z = -gridSize + i * step;
        uint32_t base = static_cast<uint32_t>(wd.positions.size() / 3);
        emitVertex(wd, {-gridSize, y, z}, c);
        emitVertex(wd, { gridSize, y, z}, c);
        wd.indices.push_back(base);
        wd.indices.push_back(base + 1);
    }
}

RigidBodyWorld::WireData RigidBodyWorld::buildWireData() const {
    WireData wd;

    std::unordered_set<const Physics::RigidBody*> colliding;
    for (const auto& m : physicsSolver_.rigidSolver().getContacts()) {
        colliding.insert(m.bodyA);
        colliding.insert(m.bodyB);
    }

    const glm::vec4 kStatic  = {0.5f, 0.5f, 0.5f, 1.f};
    const glm::vec4 kActive  = {0.6f, 1.0f, 0.3f, 1.f};
    const glm::vec4 kContact = {1.0f, 0.3f, 0.3f, 1.f};

    for (const Physics::RigidBody* body : physicsSolver_.rigidSolver().getBodies()) {
        if (!body->shape) continue;

        glm::vec4 color;
        if (body->isStatic())
            color = kStatic;
        else if (colliding.count(body))
            color = kContact;
        else
            color = kActive;

        switch (body->shape->getType()) {
        case Physics::ShapeType::Sphere: {
            auto* ss = static_cast<Physics::SphereShape*>(body->shape);
            buildSphereWire(wd,
                glm::vec3(body->position.x, body->position.y, body->position.z),
                ss->radius, color);
            break;
        }
        case Physics::ShapeType::Box: {
            auto* bs = static_cast<Physics::BoxShape*>(body->shape);
            buildBoxWire(wd, *bs, body->position, body->orientation, color);
            break;
        }
        case Physics::ShapeType::Plane: {
            auto* ps = static_cast<Physics::PlaneShape*>(body->shape);
            buildFloorWire(wd, ps->offset);
            break;
        }
        }
    }

    return wd;
}

} // namespace Phantom
