#include "HairFollowers.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Phantom::Physics {
namespace {
bool finite(const Math::Vector3df& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}

bool HairFollowers::initialize(const HairStrands& guides, const std::vector<HairRootPose>& roots,
                              size_t particlesPerStrand) {
    if (particlesPerStrand < 2 || particlesPerStrand > 4096 || roots.size() > 100000 ||
        roots.size() > 1000000/particlesPerStrand ||
        (!roots.empty() && guides.strandCount() == 0)) return false;
    HairFollowers next;
    next.source_ = &guides;
    next.revision_ = guides.revision();
    next.sourceParticles_ = guides.particleCount();
    next.particlesPerStrand_ = particlesPerStrand;
    for (const auto& r : guides.ranges()) next.referenceRoots_.push_back(r.root);
    next.bindings_.reserve(roots.size());
    for (auto root : roots) {
        const float norm = glm::dot(root.rotation,root.rotation);
        if (!finite(root.position) || !std::isfinite(norm) || norm < 1.e-12f) return false;
        root.rotation = glm::normalize(root.rotation);
        HairFollowerBinding binding;
        binding.root = root;
        std::array<float,3> distances;
        distances.fill(std::numeric_limits<float>::infinity());
        for (size_t s = 0; s < guides.strandCount(); ++s) {
            const float distance = glm::length(root.position-guides.ranges()[s].root.position);
            if (!std::isfinite(distance)) return false;
            for (size_t k = 0; k < 3; ++k) {
                if (distance < distances[k]) {
                    for (size_t l = 2; l > k; --l) {
                        distances[l] = distances[l-1];
                        binding.guides[l] = binding.guides[l-1];
                    }
                    distances[k] = distance;
                    binding.guides[k] = s;
                    break;
                }
            }
        }
        if (distances[0] <= 1.e-6f) binding.weights[0] = 1.f;
        else {
            float sum = 0.f;
            for (size_t k = 0; k < 3; ++k) {
                binding.weights[k] = std::isfinite(distances[k]) ? distances[0]/distances[k] : 0.f;
                sum += binding.weights[k];
            }
            for (auto& weight : binding.weights) weight /= sum;
        }
        next.bindings_.push_back(binding);
    }
    next.scratch_.resize(roots.size()*particlesPerStrand);
    if (!next.update(guides)) return false;
    next.scratch_.resize(next.positions_.size());
    *this = std::move(next);
    return true;
}

bool HairFollowers::update(const HairStrands& guides) {
    if (!source_) return bindings_.empty();
    if (source_ != &guides || revision_ != guides.revision() ||
        sourceParticles_ != guides.particleCount() || referenceRoots_.size() != guides.strandCount())
        return false;
    const auto& positions = guides.particles().positions;
    for (const auto& point : positions) if (!finite(point)) return false;
    for (size_t f = 0; f < bindings_.size(); ++f) {
        const auto& b = bindings_[f];
        Math::Vector3df root(0.f);
        std::array<Math::Quaternion,3> alignments;
        for (size_t k = 0; k < 3; ++k) {
            if (b.weights[k] == 0.f) continue;
            const auto& r = guides.ranges()[b.guides[k]];
            const auto& initial = referenceRoots_[b.guides[k]];
            const auto delta = r.root.rotation*glm::conjugate(initial.rotation);
            root += b.weights[k]*(positions[r.offset]+delta*(b.root.position-initial.position));
            alignments[k] = delta*b.root.rotation*glm::conjugate(r.root.rotation);
        }
        for (size_t j = 0; j < particlesPerStrand_; ++j) {
            auto point = root;
            if (j) for (size_t k = 0; k < 3; ++k) {
                if (b.weights[k] == 0.f) continue;
                const auto& r = guides.ranges()[b.guides[k]];
                const double coordinate = static_cast<double>(j)*(r.count-1)/(particlesPerStrand_-1);
                const size_t a = std::min(static_cast<size_t>(coordinate),r.count-1);
                const size_t c = std::min(a+1,r.count-1);
                const auto sample = glm::mix(positions[r.offset+a],positions[r.offset+c],
                                             static_cast<float>(coordinate-a));
                point += b.weights[k]*(alignments[k]*(sample-positions[r.offset]));
            }
            if (!finite(point)) return false;
            scratch_[f*particlesPerStrand_+j] = point;
        }
    }
    positions_.swap(scratch_);
    scratch_.resize(positions_.size());
    return true;
}
} // namespace Phantom::Physics
