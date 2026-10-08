#include "HairGenerator.h"
#include <cmath>

namespace Phantom::Physics {
bool generateHairBundle(HairStrands& output, const HairGeneratorParams& p) {
    if (p.strands < 1 || p.strands > 10000 || p.particlesPerStrand < 2 ||
        p.particlesPerStrand > 4096 || p.strands > 1000000 / p.particlesPerStrand ||
        !std::isfinite(p.length) || p.length < 1.e-4f || p.length > 100.f ||
        !std::isfinite(p.spacing) || p.spacing < 0.f || p.spacing > 10.f ||
        !std::isfinite(p.curvature) || std::abs(p.curvature) > 10.f) return false;
    const int columns = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(p.strands))));
    const int rows = (p.strands + columns - 1) / columns;
    const float norm = glm::dot(p.root.rotation, p.root.rotation);
    if (!std::isfinite(norm) || norm < 1.e-12f) return false;
    const auto rotation = glm::normalize(p.root.rotation);
    std::vector<HairStrandInput> inputs;
    inputs.reserve(p.strands);
    for (int s = 0; s < p.strands; ++s) {
        HairStrandInput in;
        in.root = p.root;
        in.root.position += rotation * Math::Vector3df(
            (s % columns - (columns - 1) * 0.5f) * p.spacing, 0.f,
            (s / columns - (rows - 1) * 0.5f) * p.spacing);
        for (int j = 0; j < p.particlesPerStrand; ++j) {
            const float t = static_cast<float>(j) / (p.particlesPerStrand - 1);
            in.restPositions.push_back({p.curvature * p.length * t * t, -p.length * t, 0.f});
        }
        // length is the discrete arc length, rather than the vertical extent.
        float arcLength = 0.f;
        for (size_t j = 1; j < in.restPositions.size(); ++j)
            arcLength += glm::length(in.restPositions[j] - in.restPositions[j-1]);
        for (auto& point : in.restPositions) point *= p.length / arcLength;
        inputs.push_back(std::move(in));
    }
    return output.initialize(inputs);
}
} // namespace Phantom::Physics
