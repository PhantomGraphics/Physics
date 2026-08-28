#include "PlaneCollider.h"
#include "PlaneBoundary.h"

namespace Phantom {
namespace Physics {

void PlaneCollider::resolve(SoftParticleSoA& particles, size_t i) const {
    particles.predicted[i] = PlaneBoundary(normal, offset).clampPosition(particles.predicted[i]);
}

} // namespace Physics
} // namespace Phantom
