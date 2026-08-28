#include "pch.h"
#include "BoundaryPsi.h"

namespace Phantom {
namespace Physics {

void computeBoundaryPsi(const std::vector<Math::Vector3df>& positions,
                         const SPHKernel& kernel,
                         float restDensity,
                         std::vector<float>& psiOut) {
    const size_t n = positions.size();
    psiOut.resize(n);
    for (size_t k = 0; k < n; ++k) {
        float sumW = kernel.getCubicSpline(0.f);
        for (size_t l = 0; l < n; ++l) {
            if (l == k) continue;
            const float dist = Math::getDistance(positions[k], positions[l]);
            sumW += kernel.getCubicSpline(dist);
        }
        psiOut[k] = (sumW > 1.0e-8f) ? (restDensity / sumW) : 0.f;
    }
}

} // namespace Physics
} // namespace Phantom
