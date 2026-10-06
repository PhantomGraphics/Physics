#pragma once

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <numeric>
#include <utility>
#include <vector>

namespace Phantom {

/** Light-ray optical depth through the existing smoke puffs, without a density
 * volume. A BVH skips puffs outside the ray. The receiving puff is excluded:
 * its own attenuation is evaluated at each generated PBVR particle on the GPU.
 * External attenuation is sampled at the puff centre (a shading approximation).
 */
class FlameSmokeShadow {
public:
    struct Puff { glm::vec3 centre; float diameter; float tau; };

    void build(std::vector<Puff> puffs) {
        puffs_ = std::move(puffs);
        order_.resize(puffs_.size());
        std::iota(order_.begin(), order_.end(), 0u);
        nodes_.clear();
        if (!puffs_.empty()) buildNode(0, static_cast<unsigned>(puffs_.size()));
    }

    // toLight must be unit length. Returns optical depth, not opacity.
    float opticalDepth(unsigned receiver, const glm::vec3& toLight) const {
        if (receiver >= puffs_.size() || nodes_.empty()) return 0.0f;
        return trace(0, puffs_[receiver].centre, toLight, receiver);
    }

    static float rayDepth(const Puff& puff, const glm::vec3& origin, const glm::vec3& direction) {
        if (puff.diameter <= 0.0f || puff.tau <= 0.0f) return 0.0f;
        const glm::vec3 delta = puff.centre - origin;
        const float along = glm::dot(delta, direction);
        const float radius = 0.5f * puff.diameter;
        const float discriminant = radius * radius - (glm::dot(delta, delta) - along * along);
        if (discriminant <= 0.0f) return 0.0f;
        const float halfChord = std::sqrt(discriminant);
        const float length = std::max(0.0f, along + halfChord - std::max(0.0f, along - halfChord));
        return puff.tau * length / puff.diameter;
    }

private:
    struct Node { glm::vec3 lo, hi; unsigned begin, end, left = 0, right = 0; };
    std::vector<Puff> puffs_;
    std::vector<unsigned> order_;
    std::vector<Node> nodes_;

    unsigned buildNode(unsigned begin, unsigned end) {
        glm::vec3 lo(FLT_MAX), hi(-FLT_MAX);
        for (unsigned i = begin; i < end; ++i) {
            const auto& p = puffs_[order_[i]];
            const glm::vec3 r(std::max(0.0f, p.diameter) * 0.5f);
            lo = glm::min(lo, p.centre - r); hi = glm::max(hi, p.centre + r);
        }
        const unsigned index = static_cast<unsigned>(nodes_.size());
        nodes_.push_back({lo, hi, begin, end});
        if (end - begin > 8) {
            const glm::vec3 extent = hi - lo;
            const int axis = extent.x > extent.y ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2);
            const unsigned mid = begin + (end - begin) / 2;
            std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                [&](unsigned a, unsigned b) { return puffs_[a].centre[axis] < puffs_[b].centre[axis]; });
            const unsigned left = buildNode(begin, mid);
            const unsigned right = buildNode(mid, end);
            nodes_[index].left = left; nodes_[index].right = right;
        }
        return index;
    }

    static bool intersects(const Node& n, const glm::vec3& origin, const glm::vec3& direction) {
        float nearDistance = 0.0f, farDistance = FLT_MAX;
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(direction[axis]) < 1.0e-8f) {
                if (origin[axis] < n.lo[axis] || origin[axis] > n.hi[axis]) return false;
            } else {
                float a = (n.lo[axis] - origin[axis]) / direction[axis];
                float b = (n.hi[axis] - origin[axis]) / direction[axis];
                if (a > b) std::swap(a, b);
                nearDistance = std::max(nearDistance, a); farDistance = std::min(farDistance, b);
                if (nearDistance > farDistance) return false;
            }
        }
        return true;
    }

    float trace(unsigned index, const glm::vec3& origin, const glm::vec3& direction, unsigned receiver) const {
        const Node& n = nodes_[index];
        if (!intersects(n, origin, direction)) return 0.0f;
        if (n.left != 0) return trace(n.left, origin, direction, receiver) + trace(n.right, origin, direction, receiver);
        float tau = 0.0f;
        for (unsigned i = n.begin; i < n.end; ++i) {
            if (order_[i] != receiver) tau += rayDepth(puffs_[order_[i]], origin, direction);
        }
        return tau;
    }
};

} // namespace Phantom
