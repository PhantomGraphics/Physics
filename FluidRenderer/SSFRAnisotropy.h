#pragma once

// Host-side glue between Phantom::Physics::computeAnisotropy() (PhysicsCore)
// and SSFluidRenderer's ellipsoid splats. Header-only on purpose: the
// renderer library (FluidRendererCore) stays independent of PhysicsCore, and
// every app that hosts SSFR already links PhysicsCore for its solvers.

#include "SSFREllipsoid.h"
#include "../Physics/AnisotropicKernel.h"

#include <chrono>
#include <cstdint>
#include <vector>

namespace Phantom {

// User-facing knobs of the anisotropic kernel. Lengths are in multiples of
// the particle radius so one setting works at any scene scale.
struct SSFRKernelSettings {
    float searchScale   = 6.0f;   // PCA neighbourhood radius / particle radius (3 spacings:
                                  // edge particles still reach N_eps neighbours)
    float maxRatio      = 4.0f;   // k_r
    float isolatedScale = 0.5f;   // k_n
    int   minNeighbors  = 25;     // N_eps (incl. the particle itself)
    float smoothing     = 0.95f;  // lambda (centre smoothing)

    Physics::AnisotropyParams toParams(float particleRadius) const {
        Physics::AnisotropyParams p;
        p.searchRadius  = searchScale * particleRadius;
        p.maxRatio      = maxRatio;
        p.isolatedScale = isolatedScale;
        p.minNeighbors  = minNeighbors;
        p.smoothing     = smoothing;
        return p;
    }
};

struct SSFRKernelStats {
    float    computeMs        = 0.0f;
    float    meanStretchRatio = 1.0f;
    uint32_t particleCount    = 0;
    uint32_t anisotropicCount = 0;
};

// Builds the per-particle centre/axes arrays SSFluidRenderer consumes.
// Keeps its scratch buffers between calls (no per-frame reallocation once
// the particle count is stable). Not thread-safe; one builder per producer.
class SSFRAnisotropyBuilder {
public:
    /// positions: particle centres. radius: world radius written to centre.w
    /// (the renderer multiplies it by its surface factor, like the sprites).
    SSFRKernelStats build(const std::vector<glm::vec3>& positions, float radius,
                          const SSFRKernelSettings& settings,
                          std::vector<glm::vec4>& outCenters,
                          std::vector<SSFREllipsoidAxes>& outAxes)
    {
        const auto t0 = std::chrono::steady_clock::now();
        Physics::computeAnisotropy(positions, settings.toParams(radius), result_);

        const size_t n = positions.size();
        outCenters.resize(n);
        outAxes.resize(n);
        for (size_t i = 0; i < n; ++i) {
            outCenters[i] = glm::vec4(result_.centers[i], radius);
            const auto& a = result_.axes[i];
            outAxes[i].axis[0] = glm::vec4(a[0], 0.0f);
            outAxes[i].axis[1] = glm::vec4(a[1], 0.0f);
            outAxes[i].axis[2] = glm::vec4(a[2], 0.0f);
        }

        SSFRKernelStats stats;
        stats.computeMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        stats.meanStretchRatio = result_.meanStretchRatio;
        stats.particleCount    = static_cast<uint32_t>(n);
        stats.anisotropicCount = static_cast<uint32_t>(result_.anisotropicCount);
        return stats;
    }

private:
    Physics::AnisotropyResult result_;
};

} // namespace Phantom
