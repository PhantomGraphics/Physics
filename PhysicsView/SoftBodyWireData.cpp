#include "SoftBodyWireData.h"

#include "../Core/ICollisionShape.h"
#include "../Core/RigidBody.h"

#include <glm/glm.hpp>

#include <array>
#include <cmath>

namespace Phantom {

namespace {

struct LineSink {
    SoftBodyWireData& out;

    void line(const glm::vec3& a, const glm::vec3& b, const glm::vec4& c) {
        vertex(a, c);
        vertex(b, c);
    }

    void vertex(const glm::vec3& p, const glm::vec4& c) {
        out.indices.push_back(static_cast<uint32_t>(out.indices.size()));
        out.positions.push_back(p.x);
        out.positions.push_back(p.y);
        out.positions.push_back(p.z);
        out.colors.push_back(c.r);
        out.colors.push_back(c.g);
        out.colors.push_back(c.b);
        out.colors.push_back(c.a);
    }
};

// A circle of `radius` around `center` in the plane spanned by axis1/axis2, as a line list.
void appendCircle(LineSink& sink, const glm::vec3& center, float radius,
                  const glm::vec3& axis1, const glm::vec3& axis2, const glm::vec4& color,
                  int segments = 32)
{
    constexpr float kTwoPi = 6.28318530717958647692f;
    for (int i = 0; i < segments; ++i) {
        const float t0 = kTwoPi * static_cast<float>(i)     / static_cast<float>(segments);
        const float t1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        sink.line(center + radius * (std::cos(t0) * axis1 + std::sin(t0) * axis2),
                  center + radius * (std::cos(t1) * axis1 + std::sin(t1) * axis2), color);
    }
}

// A wire sphere drawn as three orthogonal great circles.
void appendSphere(LineSink& sink, const glm::vec3& center, float radius)
{
    const glm::vec4 color{ 0.7f, 0.7f, 0.8f, 1.f };
    appendCircle(sink, center, radius, { 1.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, color);  // XY
    appendCircle(sink, center, radius, { 1.f, 0.f, 0.f }, { 0.f, 0.f, 1.f }, color);  // XZ
    appendCircle(sink, center, radius, { 0.f, 1.f, 0.f }, { 0.f, 0.f, 1.f }, color);  // YZ
}

// The 12 edges of a box from its 8 corners (BoxShape::getWorldCorners order).
void appendBox(LineSink& sink, const std::array<Math::Vector3df, 8>& c)
{
    static constexpr int edges[12][2] = {
        { 0, 1 }, { 0, 2 }, { 0, 4 }, { 1, 3 }, { 1, 5 }, { 2, 3 },
        { 2, 6 }, { 3, 7 }, { 4, 5 }, { 4, 6 }, { 5, 7 }, { 6, 7 },
    };
    const glm::vec4 color{ 0.6f, 0.5f, 0.9f, 1.f };
    for (const auto& e : edges)
        sink.line(glm::vec3(c[e[0]]), glm::vec3(c[e[1]]), color);
}

} // namespace

SoftBodyWireData buildSoftBodyWireData(const Physics::SoftBodySolver& solver)
{
    SoftBodyWireData data;
    LineSink sink{ data };

    for (const Physics::ISoftBody* body : solver.bodies()) {
        const auto& mesh = body->getMesh();
        for (const auto& edge : mesh.edges) {
            const auto& pa = mesh.particles.positions[edge.a];
            const auto& pb = mesh.particles.positions[edge.b];

            const float stretch = (edge.restLength > 1e-8f)
                ? glm::length(pb - pa) / edge.restLength : 1.f;
            const float r = glm::clamp(stretch - 1.f, 0.f, 1.f);
            const float b = glm::clamp(1.f - stretch, 0.f, 1.f);
            sink.line(glm::vec3(pa), glm::vec3(pb), { r, 1.f - r - b, b, 1.f });
        }
    }

    const auto& params = solver.params();
    if (params.sphereEnabled)
        appendSphere(sink, glm::vec3(params.sphereCenter), params.sphereRadius);

    for (const Physics::RigidBody* rb : solver.rigidColliderBodies()) {
        if (!rb || !rb->shape) continue;
        if (rb->shape->getType() == Physics::ShapeType::Box) {
            const auto* box = static_cast<const Physics::BoxShape*>(rb->shape);
            appendBox(sink, box->getWorldCorners(rb->position, rb->orientation));
        }
    }

    return data;
}

} // namespace Phantom
