#include "HairStrands.h"
#include <cmath>
#include <utility>

namespace Phantom::Physics {
namespace {
bool finite(const Math::Vector3df& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}

bool HairStrands::initialize(const std::vector<HairStrandInput>& inputs) {
    size_t total = 0;
    for (const auto& in : inputs) {
        if (in.restPositions.size() < 2 || in.restPositions.size() > 4096 ||
            in.restPositions.size() > 1000000 - total ||
            !std::isfinite(in.inverseMass) || in.inverseMass < 1.e-6f || in.inverseMass > 1.e6f ||
            !finite(in.root.position)) return false;
        const auto& q = in.root.rotation;
        const float norm = glm::dot(q, q);
        if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(norm) || norm < 1.e-12f) return false;
        if (!finite(in.restPositions.front()) || glm::length(in.restPositions.front()) > 1.e-6f)
            return false;
        for (size_t i = 0; i < in.restPositions.size(); ++i) {
            const auto& p = in.restPositions[i];
            if (!finite(p)) return false;
            if (i) {
                const float length = glm::length(p - in.restPositions[i - 1]);
                if (!std::isfinite(length) || length < 1.e-6f) return false;
            }
        }
        total += in.restPositions.size();
    }

    SoftParticleSoA particles;
    particles.resize(total);
    std::vector<HairStrandRange> ranges;
    std::vector<Math::Vector3df> rest;
    rest.reserve(total);
    size_t offset = 0;
    for (const auto& in : inputs) {
        HairRootPose root = in.root;
        root.rotation = glm::normalize(root.rotation);
        ranges.push_back({offset, in.restPositions.size(), root});
        for (size_t j = 0; j < in.restPositions.size(); ++j) {
            const auto local = j == 0 ? Math::Vector3df(0.f) : in.restPositions[j];
            const auto world = root.position + root.rotation * local;
            if (!finite(world)) return false;
            const size_t i = offset + j;
            particles.positions[i] = world;
            particles.predicted[i] = world;
            particles.velocities[i] = Math::Vector3df(0.f);
            particles.forces[i] = Math::Vector3df(0.f);
            particles.inverseMasses[i] = j == 0 ? 0.f : in.inverseMass;
            rest.push_back(local);
        }
        // Verify transformed distances too: a large translation can lose float precision.
        for (size_t j = 1; j < in.restPositions.size(); ++j) {
            const float length = glm::length(particles.positions[offset+j] - particles.positions[offset+j-1]);
            if (!std::isfinite(length) || length < 1.e-6f) return false;
        }
        offset += in.restPositions.size();
    }
    particles_ = std::move(particles);
    ranges_ = std::move(ranges);
    restPositions_ = std::move(rest);
    ++revision_;
    return true;
}
bool HairStrands::setDynamicState(const std::vector<Math::Vector3df>& positions,
                                 const std::vector<Math::Vector3df>& velocities) {
    if (positions.size() != particleCount() || velocities.size() != particleCount()) return false;
    for (size_t i = 0; i < positions.size(); ++i)
        if (!finite(positions[i]) || !finite(velocities[i])) return false;
    for (const auto& r : ranges_)
        if (glm::length(positions[r.offset]-r.root.position) > 1.e-5f ||
            glm::length(velocities[r.offset]) > 1.e-5f) return false;
    particles_.positions = particles_.predicted = positions;
    particles_.velocities = velocities;
    for (auto& f : particles_.forces) f = Math::Vector3df(0.f);
    return true;
}
} // namespace Phantom::Physics
